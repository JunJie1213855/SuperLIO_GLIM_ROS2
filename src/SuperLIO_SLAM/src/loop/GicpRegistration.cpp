#include "loop/SubmapRegistration.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <Eigen/Eigenvalues>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

namespace super_lio_slam
{
  namespace
  {
    class InspectableGicp : public pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>
    {
    public:
      const auto &targetCovariances() const { return target_covariances_; }
    };
  }

  // 精配准保持 source->target 的变换方向；GICP 利用局部协方差约束点云对齐。
  // coarse_pass 只允许进入本函数，最终还需覆盖率、残差及几何可观性验收。
  RegistrationResult refineRegister(const RegistrationCloud::ConstPtr &source,
                                    const RegistrationCloud::ConstPtr &target,
                                    const RegistrationCloud::ConstPtr &current,
                                    const RegistrationSettings &s,
                                    RegistrationResult out)
  {
    if (!out.accepted)
      return out;
    // 先撤销粗配准的通过标记，任何精配准中途失败都必须返回拒绝结果。
    out.accepted = false;
    const auto begin = std::chrono::steady_clock::now();
    auto finish = [&](const std::string &reason)
    {
      out.reason = reason;
      out.fine_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
      return out;
    };
    if (!validRigidTransform(out.transform))
      return finish("invalid_initial_transform");
    if (!source || !target || !current || current->empty() ||
        source->size() < static_cast<std::size_t>(s.min_points) || target->size() < static_cast<std::size_t>(s.min_points) ||
        std::min(source->size(), target->size()) <= static_cast<std::size_t>(s.covariance_neighbors))
      return finish("few_gicp_points");

    // 粗配准作为初始值
    const Eigen::Matrix4d coarse = out.transform;

    // gicp 精配准
    InspectableGicp gicp;
    // 设置输入输出
    gicp.setInputSource(source);
    gicp.setInputTarget(target);

    // 设置迭代参数
    gicp.setMaximumIterations(s.max_iterations);
    gicp.setCorrespondenceRandomness(s.covariance_neighbors);
    gicp.setMaxCorrespondenceDistance(s.correspondence_distance);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    RegistrationCloud aligned;

    // 开始对齐
    gicp.align(aligned, coarse.cast<float>());
    // PCL 返回的已是含粗初值的总变换，不能再次乘 coarse，否则会发生重复变换。
    out.transform = gicp.getFinalTransformation().cast<double>(); // already includes coarse initial guess
    if (!gicp.hasConverged() || aligned.empty() || !validRigidTransform(out.transform))
      return finish("gicp_not_converged");
    // 限制的是粗解到精解的修正量，而不是传感器距地图原点的绝对位置/角度。
    const Eigen::Matrix4d correction = coarse.inverse() * out.transform;
    if (correction.block<3, 1>(0, 3).norm() > s.max_translation ||
        Eigen::AngleAxisd(correction.block<3, 3>(0, 0)).angle() * 180.0 / M_PI > s.max_rotation_deg)
      return finish("excessive_refinement");
    pcl::KdTreeFLANN<pcl::PointXYZI> tree;
    tree.setInputCloud(target);
    // 用固定半径内最近邻独立评分，不直接采用 GICP 的内部收敛标志作为质量结论。
    // KD-tree 返回平方距离：阈值需平方，mse 的单位为 m^2，区别于粗阶段的 RMSE（m）。
    auto quality = [&](const RegistrationCloud &points, double &mse)
    {
      std::size_t count = 0;
      double sum = 0;
      std::vector<int> ids(1);
      std::vector<float> distances(1);
      for (const auto &point : points)
        if (tree.nearestKSearch(point, 1, ids, distances) == 1 &&
            distances[0] <= s.quality_radius * s.quality_radius)
        {
          ++count;
          sum += distances[0];
        }
      mse = count ? sum / count : std::numeric_limits<double>::infinity();
      return count;
    };
    out.inliers = quality(aligned, out.mse);
    out.overlap = static_cast<double>(out.inliers) / aligned.size();
    // 子图整体重合还不足以证明当前帧定位正确，另查当前扫描，防止只有旧帧“撑高”覆盖率。
    // 调用方必须保证 current 与 source 已处于同一源坐标系。
    RegistrationCloud current_aligned;
    pcl::transformPointCloud(*current, current_aligned, out.transform.cast<float>().eval());
    double current_mse = 0;
    out.current_overlap = static_cast<double>(quality(current_aligned, current_mse)) / current_aligned.size();
    if (out.inliers < static_cast<std::size_t>(s.min_inliers) || out.overlap <= s.min_overlap || out.current_overlap <= s.min_current_overlap)
      return finish("low_overlap");
    if (!std::isfinite(out.mse) || out.mse >= s.max_mse)
      return finish("high_inlier_mse");
    // Conservative geometry diagnostic, not a calibrated GICP covariance.
    // Point-to-plane information is normalized by source extent to balance units.
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(aligned, centroid);
    double scale_squared = 0;
    for (const auto &p : aligned)
      scale_squared += (p.getVector3fMap() - centroid.head<3>()).squaredNorm();
    const double scale = std::sqrt(scale_squared / aligned.size());
    if (scale < 1e-6)
      return finish("degenerate_geometry");
    // 由目标协方差的最小特征向量近似局部法向，累加归一化的点到面 J*J^T。
    // 这只是几何退化诊断，不作为经过标定的配准协方差交给 PGO/ESKF。
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    std::vector<int> ids(1);
    std::vector<float> distances(1);
    const auto covariances = gicp.targetCovariances();
    if (!covariances || covariances->size() != target->size())
      return finish("missing_covariances");
    for (const auto &p : aligned)
    {
      if (tree.nearestKSearch(p, 1, ids, distances) != 1 || distances[0] > s.quality_radius * s.quality_radius)
        continue;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen((*covariances)[ids[0]]);
      if (eigen.info() != Eigen::Success)
        continue;
      const Eigen::Vector3d n = eigen.eigenvectors().col(0);
      const Eigen::Vector3d q = (p.getVector3fMap() - centroid.head<3>()).cast<double>() / scale;
      Eigen::Matrix<double, 6, 1> jacobian;
      jacobian.head<3>() = q.cross(n);
      jacobian.tail<3>() = n;
      information.noalias() += jacobian * jacobian.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(information);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues()[5] <= 0)
      return finish("degenerate_geometry");
    // 最小/最大特征值比过小，说明至少一个运动方向约束很弱；低 MSE 也可能是退化解。
    out.information_ratio = std::max(0.0, eigen.eigenvalues()[0]) / eigen.eigenvalues()[5];
    if (out.information_ratio < s.min_information_ratio)
      return finish("degenerate_geometry");
    out.accepted = true;
    return finish("accepted");
  }
} // namespace super_lio_slam
