#include "loop/SubmapRegistration.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <limits>
#include <Eigen/SVD>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
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
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
      pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimation;
      normal_estimation.setInputCloud(cloud);
      normal_estimation.setRadiusSearch(s.normal_radius);
      normal_estimation.compute(*normals);
      pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setInputCloud(cloud);
      fpfh.setInputNormals(normals);
      fpfh.setRadiusSearch(s.feature_radius);
      pcl::PointCloud<pcl::FPFHSignature33> descriptors;
      fpfh.compute(descriptors);
      Features out;
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
      const auto centered = (points.colwise() - points.rowwise().mean()).eval();
      const Eigen::Vector3d sv = Eigen::JacobiSVD<Eigen::MatrixXd>(centered).singularValues();
      return (points.rowwise().maxCoeff() - points.rowwise().minCoeff()).norm() >= baseline &&
             sv[0] > 1e-6 && sv[1] / sv[0] > 1e-3; // allow planes, reject near-collinear inputs
    }
  } // namespace
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
    if (!source || !target || source->size() < static_cast<std::size_t>(s.min_points) ||
        target->size() < static_cast<std::size_t>(s.min_points))
      return finish("few_points");
    const auto src = features(source, s), dst = features(target, s);
    if (src.points->size() < 2 || dst.points->size() < 2)
      return finish("few_features");
    pcl::KdTreeFLANN<pcl::FPFHSignature33> src_tree, dst_tree;
    src_tree.setInputCloud(src.descriptors);
    dst_tree.setInputCloud(dst.descriptors);
    std::vector<std::pair<int, int>> pairs;
    std::vector<int> ids(2), back(1);
    std::vector<float> distances(2), reverse(1);
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
    // Spatially spread deterministic cap rather than weighting repeated local structure.
    std::vector<std::size_t> selected;
    std::vector<double> nearest(pairs.size(), std::numeric_limits<double>::infinity());
    std::vector<bool> used(pairs.size(), false);
    std::size_t next = 0;
    while (selected.size() < pairs.size() && selected.size() < static_cast<std::size_t>(s.max_pairs))
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
    params.estimate_scaling = false;
    params.noise_bound = s.noise_bound;
    params.rotation_max_iterations = s.rotation_iterations;
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.inlier_selection_mode = teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;
    params.max_clique_time_limit = s.clique_seconds;
    params.max_clique_num_threads = s.threads;
    teaser::RobustRegistrationSolver solver(params);
    solver.solve(a, b);
    const auto solution = solver.getSolution();
    out.transform.block<3, 3>(0, 0) = solution.rotation;
    out.transform.block<3, 1>(0, 3) = solution.translation;
    if (!solution.valid || !validRigidTransform(out.transform))
      return finish("invalid_teaser_transform");
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
    if (!spread(ai, s.min_baseline) || !spread(bi, s.min_baseline))
      return finish("degenerate_coarse_inliers");
    out.accepted = true;
    return finish("coarse_pass");
#endif
  }
} // namespace super_lio_slam
