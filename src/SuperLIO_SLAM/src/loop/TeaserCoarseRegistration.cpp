#include "loop/SubmapRegistration.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <limits>
#include <Eigen/SVD>
#include <omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#ifdef SUPERLIO_WITH_TEASER
#include <teaser/registration.h>
#endif

namespace super_lio_slam
{
  namespace
  {
    struct Features
    {
      RegistrationCloud::Ptr points{new RegistrationCloud};
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors{new pcl::PointCloud<pcl::FPFHSignature33>};
    };
    Features features(const RegistrationCloud::ConstPtr &cloud, const RegistrationSettings &s)
    {
      // 法向使用 normal_radius 的邻域，FPFH 使用更大的 feature_radius 邻域描述局部形状。
      // 特征匹配只给出可能的对应点，是否几何一致仍需 TEASER 和后续残差验收。
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
      pcl::PointCloud<pcl::FPFHSignature33> descriptors;
      auto compute = [&](auto &normal_estimation, auto &fpfh)
      {
        normal_estimation.setInputCloud(cloud);
        normal_estimation.setRadiusSearch(s.normal_radius);
        if (s.feature_matching == "nearest")
          normal_estimation.setViewPoint(0, 0, 0);
        normal_estimation.compute(*normals);
        fpfh.setInputCloud(cloud);
        fpfh.setInputNormals(normals);
        fpfh.setRadiusSearch(s.feature_radius);
        fpfh.compute(descriptors);
      };
      if (s.feature_matching == "nearest")
      {
        pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normal_estimation(s.threads);
        pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setNumberOfThreads(s.threads);
        compute(normal_estimation, fpfh);
      }
      else
      {
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimation;
        pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
        compute(normal_estimation, fpfh);
      }
      Features out;
      if (s.feature_matching == "nearest")
      {
        // Keep source indices, including invalid descriptors, so the uniform
        // query stride matches the independent registration tool exactly.
        *out.points = *cloud;
        *out.descriptors = std::move(descriptors);
        return out;
      }
      for (std::size_t i = 0; i < descriptors.size(); ++i)
      {
        const auto &n = (*normals)[i];
        if (!std::isfinite(n.normal_x) || !std::isfinite(n.normal_y) || !std::isfinite(n.normal_z))
          continue;
        bool valid = true;
        double magnitude = 0;
        for (float v : descriptors[i].histogram)
        {
          valid = valid && std::isfinite(v);
          magnitude += std::abs(v);
        }
        if (!valid || magnitude < 1e-6)
          continue;
        out.points->push_back((*cloud)[i]);
        out.descriptors->push_back(descriptors[i]);
      }
      return out;
    }
    bool spread(const Eigen::Matrix<double, 3, Eigen::Dynamic> &points, double baseline)
    {
      // 要求足够空间跨度且不近似共线；允许平面分布，完整 6 自由度退化检查在精配准阶段进行。
      const auto centered = (points.colwise() - points.rowwise().mean()).eval();
      const Eigen::Vector3d sv = Eigen::JacobiSVD<Eigen::MatrixXd>(centered).singularValues();
      return (points.rowwise().maxCoeff() - points.rowwise().minCoeff()).norm() >= baseline &&
             sv[0] > 1e-6 && sv[1] / sv[0] > 1e-3; // allow planes, reject near-collinear inputs
    }
  } // namespace



  // 粗配准：输入分别在 source/target 自身坐标系中，求 p_target ≈ R*p_source+t。
  // 回环时即 T_H_C，重定位时即 T_W_S；STD 候选检索不等同于这里的几何配准。
  RegistrationResult coarseRegister(const RegistrationCloud::ConstPtr &source,
                                    const RegistrationCloud::ConstPtr &target,
                                    const RegistrationSettings &s, const Eigen::Matrix4d *hint)
  {
    RegistrationResult out;
    out.used_hint = hint != nullptr;
    const auto begin = std::chrono::steady_clock::now();
    auto finish = [&](const std::string &reason)
    {
      out.reason = reason;
      out.coarse_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
      return out;
    };
#ifndef SUPERLIO_WITH_TEASER
    (void)source;
    (void)target;
    (void)s;
    (void)hint;
    return finish("teaser_not_built");
#else
    if (s.feature_matching == "nearest")
      omp_set_num_threads(s.threads);
    if (!source || !target || source->size() < static_cast<std::size_t>(s.min_points) ||
        target->size() < static_cast<std::size_t>(s.min_points))
      return finish("few_points");
    const auto src = features(source, s), dst = features(target, s);
    if (src.points->size() < 2 || dst.points->size() < 2)
      return finish("few_features");
    pcl::KdTreeFLANN<pcl::FPFHSignature33> src_tree, dst_tree;
    std::vector<std::pair<int, int>> pairs;
    std::vector<int> ids(2), back(1);
    std::vector<float> distances(2), reverse(1);
    if (s.feature_matching == "nearest")
    {
      // 全局重定位配置：单向 FPFH 最近邻，保留较宽的对应集合交给 TEASER 筛选。
      // 此分支不使用 ratio、互选或位姿提示的空间门限；相似结构产生的错误对应仍可能很多。
      // Global relocation: retain a broad pool of one-way matches and let
      // TEASER++ select the consistent subset, as in teaserpp_registration.
      auto finite = [](const pcl::FPFHSignature33 &feature)
      {
        for (const auto value : feature.histogram)
          if (!std::isfinite(value)) return false;
        return true;
      };
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr valid_target(new pcl::PointCloud<pcl::FPFHSignature33>);
      std::vector<int> target_indices;
      for (std::size_t i = 0; i < dst.descriptors->size(); ++i)
        if (finite((*dst.descriptors)[i]))
        {
          target_indices.push_back(static_cast<int>(i));
          valid_target->push_back((*dst.descriptors)[i]);
        }
      if (valid_target->size() < 3 || s.max_pairs < 3)
        return finish("few_features");
      dst_tree.setInputCloud(valid_target);
      const std::size_t stride = std::max<std::size_t>(1,
          (src.descriptors->size() + s.max_pairs - 1) / s.max_pairs);
      std::vector<int> nearest_id(1);
      std::vector<float> distance(1);
      for (std::size_t i = 0; i < src.descriptors->size(); i += stride)
        if (finite((*src.descriptors)[i]) &&
            dst_tree.nearestKSearch((*src.descriptors)[i], 1, nearest_id, distance) == 1)
          pairs.emplace_back(i, target_indices[nearest_id[0]]);
    }
    else
    {
      // 互选配置：最近/次近描述子距离比 + 反向一致性；有提示时只在提示附近搜索对应。
      // 这条分支通常更保守，不能把它的对应点数量与单向 nearest 配置直接等同比较。
      src_tree.setInputCloud(src.descriptors);
      dst_tree.setInputCloud(dst.descriptors);
      if (hint)
      {
        // Match descriptors within a broad geometric neighbourhood, rather than
        // discarding a valid second feature match because a distant wall was first.
        RegistrationCloud::Ptr transformed(new RegistrationCloud);
        pcl::transformPointCloud(*src.points, *transformed, hint->cast<float>().eval());
        pcl::KdTreeFLANN<pcl::PointXYZI> source_spatial, target_spatial;
        source_spatial.setInputCloud(transformed);
        target_spatial.setInputCloud(dst.points);
        auto local_match = [&](const pcl::PointXYZI &query, const pcl::FPFHSignature33 &feature,
                               pcl::KdTreeFLANN<pcl::PointXYZI> &tree,
                               const pcl::PointCloud<pcl::FPFHSignature33> &descriptors)
        {
          const int count = std::min<int>(64, descriptors.size());
          std::vector<int> nearby(count);
          std::vector<float> d2(count);
          const int found = tree.nearestKSearch(query, count, nearby, d2);
          double first = std::numeric_limits<double>::infinity(), second = first;
          int best = -1;
          for (int j = 0; j < found; ++j)
          {
            if (d2[j] > s.hint_radius * s.hint_radius)
              continue;
            double distance = 0;
            for (int k = 0; k < 33; ++k)
            {
              const double delta = feature.histogram[k] - descriptors[nearby[j]].histogram[k];
              distance += delta * delta;
            }
            if (distance < first)
            {
              second = first;
              first = distance;
              best = nearby[j];
            }
            else if (distance < second)
              second = distance;
          }
          return std::isfinite(second) && second > 1e-12 && first < s.feature_ratio * s.feature_ratio * second ? best : -1;
        };
        std::vector<int> reverse_matches(dst.points->size(), -1);
        for (std::size_t j = 0; j < dst.points->size(); ++j)
          reverse_matches[j] = local_match((*dst.points)[j], (*dst.descriptors)[j], source_spatial, *src.descriptors);
        for (std::size_t i = 0; i < src.points->size(); ++i)
        {
          const int target_index = local_match((*transformed)[i], (*src.descriptors)[i], target_spatial, *dst.descriptors);
          if (target_index >= 0 && reverse_matches[target_index] == static_cast<int>(i))
            pairs.emplace_back(i, target_index);
        }
      }
      else
      {
        for (std::size_t i = 0; i < src.points->size(); ++i)
        {
          if (dst_tree.nearestKSearch((*src.descriptors)[i], 2, ids, distances) != 2 ||
              distances[1] <= 1e-12 || distances[0] >= s.feature_ratio * s.feature_ratio * distances[1])
            continue;
          if (src_tree.nearestKSearch((*dst.descriptors)[ids[0]], 1, back, reverse) == 1 && back[0] == static_cast<int>(i))
            pairs.emplace_back(i, ids[0]);
        }
      }
    }
    // Spatially spread deterministic cap rather than weighting repeated local structure.
    std::vector<std::size_t> selected;
    std::vector<double> nearest(pairs.size(), std::numeric_limits<double>::infinity());
    std::vector<bool> used(pairs.size(), false);
    std::size_t next = 0;
    if (s.feature_matching == "nearest")
    {
      selected.resize(pairs.size());
      std::iota(selected.begin(), selected.end(), 0);
    }
    while (s.feature_matching != "nearest" && selected.size() < pairs.size() && selected.size() < static_cast<std::size_t>(s.max_pairs))
    {
      selected.push_back(next);
      used[next] = true;
      const auto p = (*src.points)[pairs[next].first].getVector3fMap();
      double best = -1;
      for (std::size_t j = 0; j < pairs.size(); ++j)
      {
        nearest[j] = std::min(nearest[j], static_cast<double>(((*src.points)[pairs[j].first].getVector3fMap() - p).squaredNorm()));
        if (!used[j] && nearest[j] > best)
        {
          best = nearest[j];
          next = j;
        }
      }
    }
    out.pairs = selected.size();
    if (out.pairs < static_cast<std::size_t>(s.min_pairs))
      return finish("few_fpfh_pairs");
    Eigen::Matrix<double, 3, Eigen::Dynamic> a(3, out.pairs), b(3, out.pairs);
    for (std::size_t k = 0; k < selected.size(); ++k)
    {
      const auto pair = pairs[selected[k]];
      a.col(k) = (*src.points)[pair.first].getVector3fMap().cast<double>();
      b.col(k) = (*dst.points)[pair.second].getVector3fMap().cast<double>();
    }
    if (!spread(a, s.min_baseline) || !spread(b, s.min_baseline))
      return finish("degenerate_pairs");
    teaser::RobustRegistrationSolver::Params params;
    // LiDAR 地图具有米制尺度，只估计旋转和平移；noise_bound 是 TEASER 的噪声界，单位为米。
    // 最大团模式/预算影响对应筛选耗时，GNC_TLS 用于鲁棒旋转估计。
    params.estimate_scaling = false;
    params.noise_bound = s.noise_bound;
    params.rotation_max_iterations = s.rotation_iterations;
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.inlier_selection_mode = s.exact_clique ?
        teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_EXACT :
        teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;
    params.rotation_gnc_factor = s.rotation_gnc_factor;
    params.rotation_cost_threshold = s.rotation_cost_threshold;
    params.max_clique_time_limit = s.clique_seconds;
    params.max_clique_num_threads = s.threads;
    teaser::RobustRegistrationSolver solver(params);
    solver.solve(a, b);
    const auto solution = solver.getSolution();
    out.transform.block<3, 3>(0, 0) = solution.rotation;
    out.transform.block<3, 1>(0, 3) = solution.translation;
    if (!solution.valid || !validRigidTransform(out.transform))
      return finish("invalid_teaser_transform");
    // 不直接把 solver.valid 当作成功：按 ||R*a_k+t-b_k|| 重算对应内点、比例和 RMSE。
    // RMSE 只统计阈值内的点，因此必须同时检查内点数量/比例，避免少量偶然点的低残差。
    std::vector<int> inliers;
    double squared_error = 0.0;
    for (int k = 0; k < a.cols(); ++k)
    {
      const double residual_squared =
          (solution.rotation * a.col(k) + solution.translation - b.col(k)).squaredNorm();
      if (residual_squared <= s.coarse_radius * s.coarse_radius)
      {
        inliers.push_back(k);
        squared_error += residual_squared;
      }
    }
    out.coarse_rmse = inliers.empty() ? std::numeric_limits<double>::infinity()
                                      : std::sqrt(squared_error / inliers.size());
    out.coarse_inliers = inliers.size();
    if (out.coarse_inliers < static_cast<std::size_t>(s.coarse_min_inliers) ||
        static_cast<double>(out.coarse_inliers) / out.pairs < s.coarse_min_ratio)
      return finish("few_coarse_inliers");
    if (!std::isfinite(out.coarse_rmse) || out.coarse_rmse > s.coarse_max_rmse)
      return finish("coarse_rmse_exceeded");
    Eigen::Matrix<double, 3, Eigen::Dynamic> ai(3, inliers.size()), bi(3, inliers.size());
    for (std::size_t k = 0; k < inliers.size(); ++k)
    {
      ai.col(k) = a.col(inliers[k]);
      bi.col(k) = b.col(inliers[k]);
    }
    // 初始对应点分布良好仍不够，筛选后剩下的内点也须具有足够的几何跨度。
    if (!spread(ai, s.min_baseline) || !spread(bi, s.min_baseline))
      return finish("degenerate_coarse_inliers");
    out.accepted = true;
    return finish("coarse_pass");
#endif
  }
} // namespace super_lio_slam
