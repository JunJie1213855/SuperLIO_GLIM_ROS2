// 为 lio2glim.py 生成的 dump 写出真实的 graph.bin / values.bin。
//
// 为什么需要这个：GlobalMapping::load() 对 gtsam::deserializeFromBinaryFile() 的
// 返回值不作检查（global_mapping.cpp:772-790），而该函数在文件打不开时是 return false
// 而非抛异常。所以仅"不写 graph.bin"并不会触发 recover_graph()，反而会让 isam2 拿到
// 空的 Values 并在 optimize() 时抛 out_of_range。因此这里直接构造一份合法的因子图。
//
// 结构与 recover_graph() 保持一致：
//   values : X(i) = Pose3(T_world_origin_i)
//   factors: LinearDampingFactor(X0)  +  BetweenFactor<Pos3>(X(i), X(i+1), delta)
//
// 用法: glim_graph_writer <dump目录> <位姿文件>
//   位姿文件每行: x y z qx qy qz qw   （每个子图的 T_world_origin，IMU 系）

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtsam/base/serialization.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_points/factors/linear_damping_factor.hpp>

using gtsam::symbol_shorthand::X;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "用法: %s <dump目录> <位姿文件>\n", argv[0]);
    return 1;
  }
  const std::string dir = argv[1];

  std::ifstream ifs(argv[2]);
  if (!ifs) {
    std::fprintf(stderr, "打不开位姿文件 %s\n", argv[2]);
    return 1;
  }

  std::vector<gtsam::Pose3> origins;
  double x, y, z, qx, qy, qz, qw;
  while (ifs >> x >> y >> z >> qx >> qy >> qz >> qw) {
    // GTSAM 的四元数顺序是 (w, x, y, z)
    origins.emplace_back(gtsam::Rot3::Quaternion(qw, qx, qy, qz),
                         gtsam::Point3(x, y, z));
  }
  if (origins.empty()) {
    std::fprintf(stderr, "位姿文件里没有有效位姿\n");
    return 1;
  }
  std::fprintf(stdout, "graph_writer: %zu 个子图\n", origins.size());

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  for (size_t i = 0; i < origins.size(); i++) {
    values.insert(X(i), origins[i]);
  }

  // X0 阻尼先验，与 GlobalMappingParams::init_pose_damping_scale 默认值一致
  graph.emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, 1e10);

  // 相邻子图的相对约束，与 recover_graph() 用同一种高精度噪声
  const auto noise = gtsam::noiseModel::Isotropic::Precision(6, 1e6);
  for (size_t i = 0; i + 1 < origins.size(); i++) {
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(i), X(i + 1), origins[i].between(origins[i + 1]), noise);
  }

  if (!gtsam::serializeToBinaryFile(values, dir + "/values.bin", "values")) {
    std::fprintf(stderr, "写 values.bin 失败\n");
    return 1;
  }
  if (!gtsam::serializeToBinaryFile(graph, dir + "/graph.bin", "graph")) {
    std::fprintf(stderr, "写 graph.bin 失败\n");
    return 1;
  }

  std::fprintf(stdout, "graph_writer: 已写出 graph.bin / values.bin\n");
  return 0;
}
