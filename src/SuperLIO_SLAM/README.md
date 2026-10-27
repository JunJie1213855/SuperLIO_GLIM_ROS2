# SuperLIO SLAM

This ROS 2 package contains a local copy of Super-LIO, its `basic` code, and STD. The online pipeline is:

1. Super-LIO processes LiDAR and IMU data and publishes selected keyframes after its normal observation update.
2. The backend extracts STD descriptors and searches earlier keyframes for a loop candidate.
3. ICP aligns the current keyframe with the candidate's local submap. Convergence, overlap, fitness, and correction limits gate the result.
4. Accepted ICP relative poses become robust loop factors in an iSAM2 pose graph. The backend publishes the corrected trajectory and map.

ICP supplies a pose-graph observation; it does not alter the front-end ESKF state. The front end continues publishing `world → imu`, and the backend publishes `map → world`, so TF consumers can obtain the corrected `map → imu` pose.

## Build and run

Source ROS 2 Humble and the workspace containing `livox_ros_driver2`, then build this package:

```bash
source /opt/ros/humble/setup.bash
git submodule update --init --recursive
python3 src/tools/build_teaserpp.py --jobs 1
colcon build --packages-select super_lio_slam --symlink-install
source install/setup.bash
ros2 launch super_lio_slam slam.launch.py
```

The SLAM launch defaults to `config/livox_360.yaml` for the sensor/front end and `config/slam.yaml` for keyframes and the backend. Select another copied sensor configuration with `sensor_config:=/absolute/path/to/config.yaml`. Add `rviz:=true` to start RViz.

For recorded data, use one time source for both nodes and RViz. Start the launch first, then play the bag in another terminal:

```bash
ros2 launch super_lio_slam slam.launch.py use_sim_time:=true rviz:=true
ros2 bag play /home/ros/dataset/mid360/jungle_challenge --clock
```

Restart the launch before replaying the same bag from its beginning; that also clears the pose graph and TF history. Live sensors should retain the default `use_sim_time:=false`.

The Robosense Airy `mapping` bag uses device-uptime stamps (about 11860 s) in LiDAR and IMU messages, while its `/clock` uses the recording epoch. Use the Robosense launch and `--clock` together:

```bash
ros2 launch super_lio_slam robo_slam.launch.py rviz:=true use_sim_time:=true
ros2 bag play /home/ros/dataset/robosense/mapping --clock
```

With `lio.output.dense: true`, `/lio/cloud_world_dense` publishes every complete deskewed scan transformed into `world`, independently of `lio.output.map` and `lio.output.pub_step`. `/lio/cloud_world` retains its configurable visualization behavior. These are current scans, not an accumulated map; a Unitree L1 scan in the recorded bag contains about 2100 points. The Robosense backend still uses its separately filtered keyframes. The corrected `/lio_slam/map` is independently downsampled by the backend map settings in `config/slam.yaml`.

`config/robosense_airy.yaml` enables `lio.ros.align_sensor_time_to_ros_clock`. It waits for the simulated ROS clock, then computes one offset for published odometry, clouds, keyframes, and TF; the LIO estimator still uses the original shared device time for LiDAR/IMU synchronization. Other sensor configurations leave this opt-in setting disabled.

For a Unitree UniLiDAR L1 publishing `/unilidar/cloud` and `/unilidar/imu`, use:

```bash
ros2 launch super_lio_slam unitree_slam.launch.py rviz:=true use_sim_time:=true
ros2 bag play /home/ros/dataset/unitree/L1 --clock
```

`config/unitree_l1.yaml` selects LiDAR type `9`, and `config/unitree_slam.yaml`
uses 0.2 m voxel downsampling for Unitree keyframes and backend input (`slam.keyframe.voxel_leaf: 0.2` and `keyframe_cloud_voxel_size: 0.2`). The adapter reads the
`PointCloud2` fields `x`, `y`, `z`, `intensity`, `ring`, and float32 `time`;
`time` is a per-point offset from the message header in seconds (adjust
`lio.sensor.timestamp_unit` if another Unitree driver uses milliseconds,
microseconds, or nanoseconds). The same adapter can be used for an L2 driver
with that field layout. The configured LiDAR-to-IMU extrinsic is a starting
value from the local Point-LIO L1 profile and should be calibrated for the
physical sensor.

All mapping launch profiles now include the shared `slam.launch.py` pipeline with the loop backend enabled. Use `enable_loop_backend:=false` to run only SuperLIO (keyframe publication is disabled and the default RViz view switches to `lio.rviz`). `relocation.launch.py` and `std_extract.launch.py` remain standalone tools.

## Data contract

`/lio_slam/keyframe` uses `super_lio_slam/msg/KeyFrame`. `odom_pose` is the uncorrected end-of-scan IMU pose in `world`; `cloud` is deskewed and expressed in that end-of-scan IMU frame. The keyframe ID and timestamp identify the corresponding pose and cloud. The backend publishes:

| Topic | Type | Frame |
| --- | --- | --- |
| `/lio_slam/odom` | `nav_msgs/Odometry` | `map → imu` |
| `/lio_slam/path` | `nav_msgs/Path` | `map` |
| `/lio_slam/map` | `sensor_msgs/PointCloud2` | `map` |
| `/lio_slam/loop_markers` | `visualization_msgs/MarkerArray` | `map` |
| `/lio_slam/std_triangles` | `visualization_msgs/MarkerArray` | `map` |

The SLAM RViz view draws the latest keyframe's STD triangles in cyan with yellow vertices. Set `std_visualization_max_triangles` in `config/slam.yaml` to limit clutter, or set it to `0` to hide them. The markers use corrected `map` coordinates and replace the previous keyframe's triangles.

`config/slam.yaml` contains all keyframe, STD, ICP, graph, and map parameters. `icp_fitness_threshold` is the mean squared correspondence error in square metres. `map_max_keyframes` limits the number of keyframes used when rebuilding a published map; set it to `0` for the full accumulated trajectory. `map_max_points_per_keyframe` bounds map publication cost independently of the stored registration clouds.

Each backend keyframe emits a `Loop search` summary (set `loop_diagnostic_stride: 0` to disable it, or increase the stride to reduce output). `stage` identifies the first failed gate: `few_corners`/`no_triangles`/`few_planes` for extraction; `no_hash_candidates`, `near_frames_only`, `side_length_rejected`, `vertex_rejected` for STD rough matching; `few_pairs_per_frame` for the minimum five pairs; `triangle_vote_rejected` for fewer than four consistent triangles; and `plane_score_rejected` for a plane score below `icp_threshold`. The summary includes counts at each filter, the strongest verified candidate, and active thresholds. `best` is the dense backend keyframe index, while `input` is the incoming keyframe ID. When STD passes, `ICP history -> current rejected` gives the fitness, correction, overlap, or point-count gate and its limit. A candidate must be more than `skip_near_num` backend keyframes old; both current Robosense and Unitree configurations use 10. A visual revisit alone does not prove that STD can form a valid candidate; these logs locate the rejection stage before tuning thresholds.

On the Robosense `mapping/` bag (98 seconds, 50 backend keyframes), the previous configuration produced zero triangles on 44 frames and only 1–4 triangles on the other six. Its `skip_near_num: 50` excluded every historical frame. With `slam.keyframe.voxel_leaf: 0.2`, `keyframe_cloud_voxel_size: 0.2`, `corner_thre: 2.0`, and `skip_near_num: 10`, a full replay generated roughly 300–600 triangles per frame and accepted loop 24 → 38 (STD score 0.807, ICP fitness 0.064, overlap 1.00). These tested settings are now the Robosense defaults in `config/slam.yaml`; Unitree retains its separate configuration.

For an initial check, inspect `/lio_slam/keyframe`, `/lio_slam/path`, and the `map → world → imu` TF chain while replaying a bag that revisits a place. A candidate becomes a graph factor only after the ICP gates pass; compare the corrected path and map with the front-end path after a completed loop.

The included ROS smoke test publishes five keyframes to the actual backend process. It checks a stationary revisit, a translated and rotated revisit, and an ICP rejection:

```bash
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node --moving
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node --reject-icp
```

Run these from the workspace root after sourcing `install/setup.bash`. `--pcd /absolute/path/to/binary_float32.pcd` tests with an existing XYZI scene; it repeats the same scene under known poses rather than replaying a sensor bag.

## Multi-frame STD / TEASER++ / GICP / iSAM2 backend

The optional `std_teaser_gicp` mode is implemented. The default remains `std_icp` for baseline comparison. STD still extracts descriptors from individual keyframes. Raw STD candidates enter multi-frame registration before the legacy triangle-vote/plane-score pose gates.

A `std::deque` retains at most four current keyframes, bounded by six seconds and six metres of cumulative travel. Each cloud is transformed with `O_current.inverse() * O_neighbor`; the current keyframe is the identity in that local coordinate system. Historical causal windows use the same rules and retain their own historical centre as the identity. Windows sharing any frame are rejected. Stored single-frame clouds are retained for map reconstruction and GLIM export.

The two submaps generate reciprocal, ratio-filtered FPFH correspondences. When STD can supply a geometric hint, a broad 3 m consistency filter reduces repeated-structure ambiguity; a rejected hinted registration retries without the hint, and no STD plane score is used as a new-mode admission gate. Deterministic spatial sampling bounds the TEASER++ input to 200 unique pairs. TEASER++ uses fixed scale, GNC_TLS and PMC_HEU. GICP receives the original source submap and the coarse transform as its initial guess, and returns the complete historical-from-current transform. iSAM2 consumes this as a robust BetweenFactor and updates all keyframe poses, the map and map→world TF as before.

Validation checks include unique correspondence count and spatial spread, coarse residual inliers, GICP convergence, finite rigid transforms, refinement limits, independent NN inlier MSE/coverage, current-frame coverage, and a normalized point-to-plane information eigenvalue ratio. That last check is a conservative geometry diagnostic, not a calibrated pose covariance. One plane or an otherwise weakly constrained scene can be rejected even with low residuals.

Build TEASER++ first using the workspace helper; its PMC, tinyply and Spectra sources are pinned, and are cached under `build/teaserpp_vendor`. A populated cache supports `--offline`:

```bash
git submodule update --init --recursive
python3 src/tools/build_teaserpp.py --jobs 1
source /opt/ros/humble/setup.bash
# Also source the workspace providing livox_ros_driver2.
MAKEFLAGS=-j1 colcon build --base-paths src --packages-select super_lio_slam --symlink-install \
  --cmake-args -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_registration_mode:=std_teaser_gicp use_sim_time:=true rviz:=true
```

The same mode argument is supported by `slam.launch.py` and `robo_slam.launch.py`. `registration_config` defaults to [config/loop_teaser_gicp.yaml](config/loop_teaser_gicp.yaml); override that argument to tune the new mode. Existing STD age/cooldown and iSAM2 noise parameters still come from the sensor SLAM YAML. Switching to `loop_registration_mode:=std_icp` restores the old registration path. To build without TEASER++, use `-DSUPERLIO_WITH_TEASER=OFF`; attempting the new mode then fails explicitly at startup.

Both registration modes now use an asynchronous registration worker and a separate map worker. STD generation, candidate verification and descriptor insertion stay sequential in the keyframe callback; each valid ingested keyframe is retained and gets an odometry graph node. Registration consumes immutable cloud/pose snapshots. `registration_queue_capacity` (default 2) bounds pending detection tasks; overload evicts the oldest pending task, never its keyframe or graph node. A running solver is not interrupted. Late constraints connect existing graph nodes and refresh the latest pose/path/correction, rather than publishing the old task frame as current. iSAM2 updates are serialized by a shared graph mutex, so graph optimization can briefly delay ingress. Map requests coalesce to the newest pending pose snapshot, with one reconstruction in progress. Historical feature caching and incremental map reconstruction are not implemented.

Normal logs report STD triangle matches and registration quality decisions, plus throttled (5 s) stage timing: cloud filtering, descriptor generation, search/verification/insertion, ingestion and sensor-stamp age, registration queue wait/work and map rebuild. Queue eviction counts are also throttled. A coarse candidate must pass inlier count/fraction, geometry and `coarse_max_inlier_rmse_m` (default 0.3 m); this residual is not ground-truth pose accuracy. Fine registration retains its coverage/MSE and observability gates. Search details/retries use DEBUG. The soft registration budget prevents starting additional candidates; it does not interrupt a running solver.

The asynchronous overload integration test is `test/async_backend_smoke.py --backend <loop_backend_node>`. It publishes a burst, verifies every input is retained in the graph, checks bounded task eviction, delayed-loop publication at the latest frame and asynchronous map output. Use a private ROS domain when running tests.

Tests after building:

```bash
ctest --test-dir build/super_lio_slam -R submap_registration_test --output-on-failure
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py \
  --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node --mode std_teaser_gicp
# Nonzero rotation with stronger local drift: use a shorter calibrated test window.
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py \
  --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node \
  --mode std_teaser_gicp --moving --submap-frames 2 --coarse-inlier-ratio 0.2
# Repeat with --reject-icp (the flag forces registration rejection in either mode).
```

Runtime parameters are experimental until validated on the deployment sensor. The design [plan](../../docs/loop_std_teaser_gicp_isam2_plan.md) and [mathematical derivation](../../docs/loop_submap_registration_math.md) explain the coordinate contract. The proposed YAML in docs is a design sketch and contains parameters not implemented by this version; use the runtime YAML above.

### Validation and limitations

The synthetic nonzero-rotation stress scene introduces 0.25 m translation drift and 0.02 rad yaw drift per keyframe. With a four-frame window the accumulated cloud is internally distorted and the default coarse inlier gate rejects it; this default stress case is not a passing test. The two-frame / 0.2-inlier-ratio experiment above passes without changing the deployed defaults. This is evidence for window/noise calibration, not a reason to relax all deployment gates. See the [recorded validation results](../../docs/loop_registration_validation.md) for exact configurations and real-bag coverage. No absolute accuracy improvement is claimed without ground truth.

## Unified launch arguments

All mapping profiles, including the legacy `M2DGR.py`, `MCD_ATH.py` and `NCLT.py` names, share these arguments:

| Argument | Default / behavior |
| --- | --- |
| `sensor_config` | Profile-specific sensor YAML; caller override is preserved |
| `slam_config` | `slam.yaml`, or `unitree_slam.yaml` for Unitree |
| `registration_config` | `loop_teaser_gicp.yaml` |
| `enable_loop_backend` | `true`; `false` also disables front-end keyframe publication |
| `loop_registration_mode` | `std_icp`; choose `std_teaser_gicp` for the new multi-frame mode |
| `use_sim_time` | `false`; applied to front end, backend and RViz |
| `rviz` | `false` in the generic launch; `true` in sensor profiles |
| `rviz_config` | `slam.rviz` with backend, `lio.rviz` in front-end-only mode |
| `log_level` / `rviz_log_level` | `info` / `warn` |

```bash
# Existing sensor launch names now support the same online loop options.
ros2 launch super_lio_slam Livox_mid360.launch.py \
  loop_registration_mode:=std_teaser_gicp use_sim_time:=true
# Run only the front end with the matching RViz coordinate frame.
ros2 launch super_lio_slam robosense_airy.launch.py enable_loop_backend:=false
```

Relocation accepts `sensor_config`, `use_sim_time`, `rviz`, `rviz_config` and logging arguments, and does not start a loop backend. Standalone STD extraction accepts `std_config`, `input_topic`, `output_dir`, a typed integer `max_visualized_descriptors`, `use_sim_time` and `log_level`. The copied entry in `std/ros2/launch` delegates to the installed `super_lio_slam` utility instead of referring to a separate `std_detector` package.

GLIM in this workspace has no Python launch files; its existing Offline Viewer entry point is unchanged.

## GTSAM runtime isolation

The loop backend is launched with the library directories selected by CMake prepended to its own `LD_LIBRARY_PATH`. This prevents ROS GTSAM 4.2 (`libgtsam.so.4`) from replacing a build against GTSAM 4.3a1 with the same SONAME. The generated `config/runtime_library_dirs.txt` is installed with the package; rebuild after moving the dependency installation. This is local to the backend process and does not change the terminal or other ROS nodes. Direct `ros2 run` / executable tests still require the matching runtime library environment.

## UrbanNav 2020-03-14-16-45-35 inspection

The local bag contains 32-ring `/velodyne_points` at about 10 Hz and `/imu/data` at about 200 Hz. `vlp32.yaml` now selects VELO32 (`lio.sensor.lidar_type: 4`). Sampled PointCloud2 messages contain x/y/z/intensity/ring but **no per-point time field**. The current Velodyne adapter expects float32 `time` in seconds, so successful launch does not establish valid deskewing for this bag. Per-point time reconstruction or a source with recorded point times is required before evaluating mapping quality; LiDAR–IMU extrinsics also need verification.

Use `use_sim_time:=true` with bag playback `--clock`. Selecting only `/velodyne_points /imu/data` avoids unrelated camera traffic and custom Novatel message types. See the [inspection report](../../docs/urbannav_dataset_check.md) for exact findings.

To hide only the repeated missing Velodyne `time` field warning, set the environment
variable before starting the front-end (default is enabled):

```bash
SUPERLIO_WARN_MISSING_TIME=0 ros2 launch super_lio_slam vlp32.launch.py \
  loop_registration_mode:=std_teaser_gicp use_sim_time:=true
```

Use `SUPERLIO_WARN_MISSING_TIME=1` or unset the variable to restore the warning;
`0`, `false` and `off` disable it. The setting is read on the first untimed cloud
and requires a node restart to change. Other PCL warnings/errors remain enabled.
Untimed points receive zero time offsets; this switch does not reconstruct point
acquisition times or restore motion deskewing accuracy.

Loop candidates must now be strictly more than 30 sensor seconds apart by default
(`loop_min_time_separation_s: 30.0`). Exactly 30 seconds is rejected. Both modes
filter near-time candidates before registration and recheck before inserting the
factor, including delayed asynchronous results. This compares the two keyframe
message stamps, independently of playback rate or worker queue delay; the existing
`loop_min_interval` keyframe cooldown still applies. Synthetic scheduling smoke
tests explicitly override the separation to zero; temporal boundary tests use 30.

Candidate retrieval defaults to spatial proximity for troubleshooting:
`loop_candidate_mode:=distance loop_candidate_radius_m:=15.0`. It searches raw LIO
positions, keeps the strict >30 sensor-second separation, skips STD descriptors,
and passes the raw relative pose as the registration prior. The same TEASER/GICP
quality gates still apply. Use `loop_candidate_mode:=std` to restore STD retrieval.
Both arguments are inherited by sensor launch profiles. Distance alone never
creates a loop edge; large odometry drift beyond the radius can hide true revisits.
