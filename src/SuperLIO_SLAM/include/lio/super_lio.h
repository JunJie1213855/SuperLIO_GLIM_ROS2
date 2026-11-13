

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <queue>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>
#include <cstdint>
#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "basic/alias.h"
#include "common/ds.h"
#include "common/timer.h"
#include "params.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"
#include "lio/AdaptiveVoxelFilter.h"
#include "ros/ROSWrapper.h"
#include <std_msgs/msg/float64_multi_array.hpp>

namespace LI2Sup
{

  class SuperLIO
  {
  public:
    SuperLIO() {};
    virtual ~SuperLIO() = default;

    void setROSWrapper(const ROSWrapper::Ptr &wrapper)
    {
      data_wrapper_ = wrapper;
    }
    virtual void init();
    virtual void process();
    void saveMap();
    void printTimeRecord();

  protected:
    void stateWaitKFInit();
    void stateWaitMapInit();
    void stateProcess();
    virtual bool kf_init();
    virtual bool map_init();
    void Propagation_Undistort();
    void DownSample();
    void Observe();
    virtual void UpdateMap();
    virtual void Output();
    void PublishKeyframe();
    void caceData();
    void ProcessCaceMap();
    void PublishObservationDiagnostics(const BASIC::SE3 &prior_pose);

  protected:
    using StateFn = void (SuperLIO::*)();
    using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
    using KNNHeapType = KNNHeap<5, BASIC::V3>;
    StateFn state_fn_;
    ESKF::Ptr kf_;
    OctVoxMapType::Ptr ivox_;
    VoxelGridClosest<BASIC::PointType> voxel_grid_fliter_;
    bool adaptive_downsample_ = false;
    std::unique_ptr<AdaptiveVoxelFilter> adaptive_voxel_filter_;
    AdaptiveVoxelStats adaptive_voxel_stats_;
    double last_adaptive_log_time_ = -1;
    std::size_t plane_observation_count_ = 0;
    struct ObservationSample
    {
      uint32_t kind = 0; // 0=未使用，1=实际采用的点到面观测；保留诊断协议类型字段。
      double residual = 0, information = 0;
      BASIC::V6d jacobian = BASIC::V6d::Zero();
    };
    std::vector<ObservationSample> observation_samples_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr observation_points_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr observation_metrics_pub_;
    ROSWrapper::Ptr data_wrapper_;
    MeasureGroup measures_;

    bool flg_init_ = false;
    bool flg_first_scan_ = true;
    std::vector<DynamicState> propagate_states_;
    BASIC::CloudPtr scan_undistort_full_;
    BASIC::CloudPtr ds_undistort_;
    BASIC::CloudPtr map_undistort_; // fixed 与观测点云共享；adaptive 为 d/2 的独立入图点云。
    BASIC::CloudPtr point_map_, world_pc_, ds_world_;
    int frame_num_ = 0;
    BASIC::SE3 sys_init_pose_;
    BASIC::SE3 last_pose_;

    std::size_t effect_knn_num_ = 0;
    BASIC::VV3 points_world_v3_, points_body_v3_;
    // 每个并行任务独立写一个字节；不能用按位打包的 vector<bool>。
    std::vector<uint8_t> effect_mask_, effect_knn_mask_;
    // 0=无有效平面，1=平面候选。
    std::vector<uint8_t> plane_candidate_mask_;
    BASIC::VV3 diagnostic_nearest_targets_; // 仅诊断使用，不参与滤波更新。
    std::vector<float> point_lengths_;
    std::vector<int> effect_knn_idxs_;
    std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
    std::vector<std::array<double, 4>> abcd_vec_;
    int pcd_index_ = -1;

    bool keyframe_enabled_ = true;
    bool have_keyframe_ = false;
    double keyframe_distance_ = 1.5;
    double keyframe_angle_rad_ = 10.0 * 3.14159265358979323846 / 180.0;
    double keyframe_max_interval_ = 3.0;
    double keyframe_voxel_leaf_ = 0.5;
    int keyframe_min_points_ = 100;
    uint32_t next_keyframe_id_ = 0;
    double last_keyframe_time_ = 0.0;
    BASIC::SE3 last_keyframe_pose_;

    Timer time_record_;
  };

} // namespace END.

#endif
