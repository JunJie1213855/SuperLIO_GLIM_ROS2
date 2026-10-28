# SuperLIO SLAM

This ROS 2 package contains a local copy of Super-LIO, its `basic` code, and STD. The online pipeline is:

1. Super-LIO processes LiDAR and IMU data and publishes selected keyframes after its normal observation update.
2. The backend retrieves earlier keyframes using STD descriptors (default) or raw-odometry distance, then checks both sensor-time separation and raw-odometry distance.
3. An asynchronous worker verifies candidates with multi-frame FPFH / TEASER++ / GICP (default), or direct GICP for comparison. Registration quality gates decide whether to accept the relative pose.
4. Accepted relative poses become robust loop factors in an iSAM2 pose graph. The backend publishes the corrected trajectory and rebuilds the map in a separate worker.

GICP supplies a pose-graph observation; it does not alter the front-end ESKF state. The front end continues publishing `world → imu`, and the backend publishes `map → world`, so TF consumers can obtain the corrected `map → imu` pose.

## Build and run

Run the following from the workspace root, not from this package directory. These paths match the local WSL Ubuntu 22.04 / ROS 2 Humble setup; adjust the workspace and Livox paths on another machine.

TEASER++ is enabled at build time by default (`SUPERLIO_WITH_TEASER=ON`); the default runtime pipeline is `loop_candidate_mode:=std loop_registration_mode:=std_teaser_gicp`. Initialize its source submodule and build/install the dependency before running colcon:

```bash
cd /home/ros/rosws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash
git submodule update --init --recursive
python3 tools/build_teaserpp.py --jobs 1
MAKEFLAGS=-j1 colcon build --packages-select super_lio_slam --symlink-install \
  --executor sequential --cmake-args \
  -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
  -DSUPERLIO_WITH_TEASER=ON -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch super_lio_slam slam.launch.py
```

The helper is `tools/build_teaserpp.py`, not `src/tools/build_teaserpp.py`. It resolves this workspace from its location, pins TEASER++ and its transitive sources, builds under `build/teaserpp`, and installs under `install/teaserpp` without sudo. The generated package configuration is `install/teaserpp/lib/cmake/teaserpp/teaserppConfig.cmake`; this package's CMake already searches that installation prefix. Source code alone does not provide the installed configuration.

For a build with only the existing GICP registration path, skip the TEASER++ helper and use:

```bash
MAKEFLAGS=-j1 colcon build --packages-select super_lio_slam --symlink-install \
  --executor sequential --cmake-args \
  -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM -DSUPERLIO_WITH_TEASER=OFF
source install/setup.bash
```

That build requires launching with `loop_registration_mode:=std_gicp`; the default `std_teaser_gicp` mode fails explicitly at startup without TEASER++. To enable the new mode later, build the dependency and rebuild with `-DSUPERLIO_WITH_TEASER=ON` explicitly, since CMake caches the option.

### Build troubleshooting

- `Could not find ... teaserppConfig.cmake`: run `python3 tools/build_teaserpp.py --jobs 1` from the workspace root before colcon. For another TEASER++ installation, pass `-Dteaserpp_DIR=/absolute/path/to/lib/cmake/teaserpp` in the colcon CMake arguments.
- `fatal: cannot change to '/home/ros/rosws/src/SuperLIO_SLAM/third_party/TEASER-plusplus'`: an older helper resolved one directory too high. In the current `tools/` layout it must use `Path(__file__).resolve().parents[1]` for the workspace root.
- Use `--offline` only after all three pinned dependency checkouts exist under `build/teaserpp_vendor`. An initial build requires network access to fetch them.
- The `glim_ros` scoped-header message from `ament_auto_package` is an installation-layout warning, not the TEASER++ configuration failure.

For direct executable tests, source the workspaces above, then prepend the build-compatible libraries while preserving the ROS library paths:

```bash
export LD_LIBRARY_PATH=/usr/local/lib:/home/ros/rosws/SuperLIO_GLIM_ROS2/install/teaserpp/lib:${LD_LIBRARY_PATH:-}
ctest --test-dir build/super_lio_slam -R submap_registration_test --output-on-failure
```

The shared SLAM launch applies the recorded GTSAM/TEASER++ library directories to the backend process automatically; see [GTSAM runtime isolation](#gtsam-runtime-isolation).

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

`config/slam.yaml` contains keyframe, STD, graph, map and direct-GICP target-window settings. Shared registration parameters are loaded from `registration_config` (default `config/loop_teaser_gicp.yaml`); `quality_max_inlier_mse_m2` limits independent nearest-neighbor inlier MSE in square metres. `map_max_keyframes` limits the number of keyframes used when rebuilding a published map; set it to `0` for the full accumulated trajectory. `map_max_points_per_keyframe` bounds map publication cost independently of the stored registration clouds.

Normal logs report registration decisions, plus stage timings throttled to 5 seconds; the old per-keyframe `Loop search` summary is no longer emitted. `loop_diagnostic_stride` remains a compatibility parameter. In STD retrieval mode, candidates must be more than `skip_near_num` backend keyframes old (10 in the current configs). Both retrieval modes also require a sensor-stamp gap strictly greater than `loop_min_time_separation_s` (30 seconds by default). `Distance candidate` means verification has started; `STD ... matched` means a retrieved candidate passed the time/distance gates; only `Loop ... accepted` means a factor was added. Registration rejection reasons identify the failed quality gate.

Historical STD/ICP Robosense replay results predate the GICP migration and are
not a validation of the current direct-GICP path. See the archived validation
reports for those measurements; current results must be measured with the selected
registration mode and its shared quality gates.

For an initial check, inspect `/lio_slam/keyframe`, `/lio_slam/path`, and the `map → world → imu` TF chain while replaying a bag that revisits a place. A candidate becomes a graph factor only after the GICP gates pass; compare the corrected path and map with the front-end path after a completed loop.

The included ROS smoke test publishes five keyframes to the actual backend process. It checks a stationary revisit, a translated and rotated revisit, and a GICP rejection:

```bash
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node --moving
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node --reject-registration
```

Run these from the workspace root after sourcing `install/setup.bash`. `--pcd /absolute/path/to/binary_float32.pcd` tests with an existing XYZI scene; it repeats the same scene under known poses rather than replaying a sensor bag.

## Multi-frame STD / TEASER++ / GICP / iSAM2 backend

The default pipeline is `loop_candidate_mode:=std` with `loop_registration_mode:=std_teaser_gicp`. STD extracts descriptors from individual keyframes and retrieves historical candidates. Candidates must pass the strict sensor-time and raw-odometry distance gates before submap registration. A triangle-consistent STD pose can supply an optional hint, but a missing hint does not reject a candidate. Neither STD triangle votes nor its legacy plane score is a hard admission gate in this mode. Direct `std_gicp` and distance retrieval remain available as comparison modes.

A `std::deque` retains at most four current keyframes, bounded by six seconds and six metres of cumulative travel. Each cloud is transformed with `O_current.inverse() * O_neighbor`; the current keyframe is the identity in that local coordinate system. Historical causal windows use the same rules and retain their own historical centre as the identity. Windows sharing any frame are rejected. Stored single-frame clouds are retained for map reconstruction and GLIM export.

The two submaps generate reciprocal, ratio-filtered FPFH correspondences. When STD can supply a geometric hint, a broad 3 m consistency filter reduces repeated-structure ambiguity; a rejected hinted registration retries without the hint, and no STD plane score is used as a new-mode admission gate. Deterministic spatial sampling bounds the TEASER++ input to 200 unique pairs. TEASER++ uses fixed scale, GNC_TLS and PMC_HEU. GICP receives the original source submap and the coarse transform as its initial guess, and returns the complete historical-from-current transform. iSAM2 consumes this as a robust BetweenFactor and updates all keyframe poses, the map and map→world TF as before.

Validation checks include unique correspondence count and spatial spread, coarse residual inliers, GICP convergence, finite rigid transforms, refinement limits, independent NN inlier MSE/coverage, current-frame coverage, and a normalized point-to-plane information eigenvalue ratio. That last check is a conservative geometry diagnostic, not a calibrated pose covariance. One plane or an otherwise weakly constrained scene can be rejected even with low residuals.

Build TEASER++ first using the workspace helper; its PMC, tinyply and Spectra sources are pinned, and are cached under `build/teaserpp_vendor`. A populated cache supports `--offline`:

```bash
cd /home/ros/rosws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash
git submodule update --init --recursive
python3 tools/build_teaserpp.py --jobs 1
MAKEFLAGS=-j1 colcon build --base-paths src --packages-select super_lio_slam --symlink-install \
  --executor sequential --cmake-args -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM \
  -DSUPERLIO_WITH_TEASER=ON -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_registration_mode:=std_teaser_gicp use_sim_time:=true rviz:=true
```

The same mode argument is supported by `slam.launch.py` and `robo_slam.launch.py`. `registration_config` defaults to [config/loop_teaser_gicp.yaml](config/loop_teaser_gicp.yaml); override that argument to tune the new mode. Existing STD age/cooldown and iSAM2 noise parameters still come from the sensor SLAM YAML. Switching to `loop_registration_mode:=std_gicp` uses direct GICP with the retrieved relative pose as its initial guess. To build without TEASER++, use `-DSUPERLIO_WITH_TEASER=OFF`; attempting the new mode then fails explicitly at startup.

Both GICP registration modes use an asynchronous registration worker and a separate map worker. STD generation, candidate verification and descriptor insertion stay sequential in the keyframe callback; each valid ingested keyframe is retained and gets an odometry graph node. Registration consumes immutable cloud/pose snapshots. `registration_queue_capacity` (default 2) bounds pending detection tasks; overload evicts the oldest pending task, never its keyframe or graph node. A running solver is not interrupted. Late constraints connect existing graph nodes and refresh the latest pose/path/correction, rather than publishing the old task frame as current. iSAM2 updates are serialized by a shared graph mutex, so graph optimization can briefly delay ingress. Map requests coalesce to the newest pending pose snapshot, with one reconstruction in progress. Historical feature caching and incremental map reconstruction are not implemented.

Normal logs report STD candidates passing the time/distance gates and registration quality decisions, plus throttled (5 s) stage timing: cloud filtering, descriptor generation, search/verification/insertion, ingestion and sensor-stamp age, registration queue wait/work and map rebuild. Queue eviction counts are also throttled. A coarse candidate must pass inlier count/fraction, geometry and `coarse_max_inlier_rmse_m` (default 0.3 m); this residual is not ground-truth pose accuracy. Fine registration retains its coverage/MSE and observability gates. Search details/retries use DEBUG. The soft registration budget prevents starting additional candidates; it does not interrupt a running solver.

The asynchronous overload integration test is `test/async_backend_smoke.py --backend <loop_backend_node>`. It publishes a burst, verifies every input is retained in the graph, checks bounded task eviction, delayed-loop publication at the latest frame and asynchronous map output. Use a private ROS domain when running tests.

Tests after building:

```bash
ctest --test-dir build/super_lio_slam -R submap_registration_test --output-on-failure
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py \
  --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node \
  --candidate-mode distance --mode std_teaser_gicp \
  --submap-frames 2 --coarse-inlier-ratio 0.2 --coarse-max-rmse 0.35
# Historical STD rotation experiment; uses explicit synthetic-test thresholds.
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py \
  --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node \
  --candidate-mode std --mode std_teaser_gicp --moving \
  --submap-frames 2 --coarse-inlier-ratio 0.2 --coarse-max-rmse 0.35
# Repeat with --reject-registration (the flag forces registration rejection in either mode).
```

Runtime parameters are experimental until validated on the deployment sensor. The design [plan](../../docs/loop_std_teaser_gicp_isam2_plan.md) and [mathematical derivation](../../docs/loop_submap_registration_math.md) explain the coordinate contract. The proposed YAML in docs is a design sketch and contains parameters not implemented by this version; use the runtime YAML above.

### Validation and limitations

The synthetic nonzero-rotation stress scene introduces 0.25 m translation drift and 0.02 rad yaw drift per keyframe. With a four-frame window the accumulated cloud is internally distorted and the default coarse inlier gate rejects it; this default stress case is not a passing test. The historical passing experiment uses two frames, a 0.2 inlier ratio and a 0.35 m coarse RMSE limit, as shown above; these are explicit test overrides and do not change deployed defaults. See the [recorded validation results](../../docs/loop_registration_validation.md) for exact configurations and real-bag coverage. No absolute accuracy improvement is claimed without ground truth.

Earlier build verification on 2026-10-28, before the GICP/default-pipeline migration: TEASER++ built and installed, `super_lio_slam` built with TEASER++ enabled, and `submap_registration_test` passed (1/1). The launch argument listing also loaded successfully. An additional online synthetic run with distance retrieval and the default four-frame / 0.3 inlier-ratio / 0.3 m RMSE settings rejected its candidate (`few_coarse_inliers`, 24/200) and failed the test's expected-loop assertion; successful compilation does not imply that this fixture passes with runtime defaults. Two subsequent runs with the two-frame / 0.2 / 0.35 m overrides timed out during ROS publisher/subscriber discovery (`Backend subscription unavailable`), including a retry in a different ROS domain; they did not reach registration, so that earlier run did not establish a passing two-frame online test. No full-bag accuracy evaluation was run in this verification.

### GICP and default-pipeline migration verification (2026-10-28)

The package build and `submap_registration_test` passed (1/1), including direct
GICP with an imperfect initial pose, transform direction, invalid-hint rejection,
sparse-input rejection, quality rejection and planar degeneracy. Online synthetic
checks used the installed `rmw_cyclonedds_cpp` middleware and separate ROS domains:

| Check | Result |
| --- | --- |
| STD -> time/radius -> TEASER++ -> GICP, default registration quality thresholds | Passed; synthetic 1 m drift reduced to about 0.0099 m |
| STD candidate beyond the configured radius | Passed; no loop factor, original drift retained |
| STD candidate exactly 30 sensor seconds apart | Passed; no loop factor, original drift retained |
| STD + direct GICP, nonzero translation / 90-degree revisit | Passed; synthetic 1 m drift reduced to about 0.0105 m |

The default-quality positive check used `--timestamp-step 10
--loop-min-time-separation 30`. Ten-second spacing trims the six-second causal
window to one frame; this is acceptance-pipeline coverage, not evidence that the
four-frame accumulated-drift stress fixture or a real sensor bag passes. Earlier
four-frame fixture failures remain documented above. No ground-truth ATE/RPE or
real-bag accuracy evaluation was performed in this migration.

To reproduce the default-quality acceptance check after sourcing the workspaces
and setting the direct-executable library path as above:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1   python3 src/SuperLIO_SLAM/test/online_loop_smoke.py   --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node   --candidate-mode std --mode std_teaser_gicp   --loop-min-time-separation 30 --timestamp-step 10
```

The final strict-quality update uses inlier MSE <0.1 m² and source/current
coverage >0.8 in both registration profiles and both GICP paths. Package build,
CTest (1/1), the default-quality STD/TEASER++/GICP synthetic positive and the
rotated STD/direct-GICP positive passed again. Boundary tests reject MSE equal to
its upper bound, source coverage exactly 80%, and current-frame coverage exactly
80%; the MSE test also accepts the immediately lower adjacent floating-point value.

## Unified launch arguments

All mapping profiles, including the legacy `M2DGR.py`, `MCD_ATH.py` and `NCLT.py` names, share these arguments:

| Argument | Default / behavior |
| --- | --- |
| `sensor_config` | Profile-specific sensor YAML; caller override is preserved |
| `slam_config` | `slam.yaml`, or `unitree_slam.yaml` for Unitree |
| `registration_config` | `loop_teaser_gicp.yaml` |
| `enable_loop_backend` | `true`; `false` also disables front-end keyframe publication |
| `loop_candidate_mode` | `std`; choose `distance` for a proximity-retrieval comparison |
| `loop_candidate_radius_m` | `15.0`; raw-odometry distance gate for both retrieval modes |
| `loop_registration_mode` | `std_teaser_gicp`; choose `std_gicp` for direct GICP |
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

Relocation accepts `sensor_config`, `relocation_config`, `map_path`, `use_sim_time`, `rviz`, `rviz_config` and logging arguments. It runs local LIO, fixed-S motion-compensated accumulation and asynchronous TEASER++/GICP, then rebases the complete KF state and switches to world-map tracking. It does not start a loop backend. See the [relocation runtime guide](../../docs/motion_compensated_relocalization_runtime.md) for commands, initialization requirements and frame semantics. Standalone STD extraction accepts `std_config`, `input_topic`, `output_dir`, a typed integer `max_visualized_descriptors`, `use_sim_time` and `log_level`. The copied entry in `std/ros2/launch` delegates to the installed `super_lio_slam` utility instead of referring to a separate `std_detector` package.

GLIM in this workspace has no Python launch files; its existing Offline Viewer entry point is unchanged.

## Commands for each loop mode

The default acceptance pipeline is:

```text
STD candidates -> sensor-time gap > 30 s AND raw-odometry distance <= 15 m
  -> FPFH + TEASER++ coarse registration -> coarse inlier count/ratio/RMSE gates
  -> GICP refinement -> independent NN distance/MSE/coverage and geometry gates
  -> robust loop factor in iSAM2 -> corrected path and map
```

The time and radius gates are checked after retrieval, before worker registration,
and again before committing a delayed constraint. Distances use raw LIO poses on
both sides. The default coarse inlier radius is 0.6 m, with at least 6 inliers,
inlier ratio at least 0.3 and inlier RMSE at most 0.3 m. GICP uses 2 m solver
correspondences, a 1 m independent quality radius, inlier MSE strictly below 0.1 m²,
source/current coverage both strictly above 0.8, and at least 100 inliers. Refinement limits
are 5 m / 20 degrees, with information-ratio floor `1e-5`; convergence, valid rigid
transforms, non-overlapping submap members and the loop cooldown are also required.
RMSE in metres and MSE in square metres are different geometric metrics.
Equality at 0.1 m² MSE or 80% coverage is rejected. No separate score-above-0.8
condition is used. PCL's `getFitnessScore()` is a mean squared distance, so lower
is better; it is not a normalized 0–1 confidence. Our independent inlier MSE
averages squared nearest-neighbor distances inside `quality_nn_radius_m`;
coverage divides that inlier count by the source/current cloud size respectively.



Candidate retrieval and registration are independent. `std_gicp` and `std_teaser_gicp`
name the registration paths; either can use distance or STD retrieval.

| Pipeline | `loop_candidate_mode` | `loop_registration_mode` |
| --- | --- | --- |
| Distance + GICP (comparison) | `distance` | `std_gicp` |
| STD + GICP | `std` | `std_gicp` |
| Distance + multi-frame TEASER++ / GICP | `distance` | `std_teaser_gicp` |
| STD + multi-frame TEASER++ / GICP (default) | `std` | `std_teaser_gicp` |

Load the environment in each launch terminal:

```bash
cd /home/ros/rosws/SuperLIO_GLIM_ROS2
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash
source install/setup.bash
```

Choose one command below. These examples use Unitree L1 bag playback. Stop the
current launch before switching modes; no rebuild is needed to switch among all
four combinations when the package was built with `SUPERLIO_WITH_TEASER=ON`.

```bash
# Distance retrieval + direct GICP: comparison mode.
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=distance loop_candidate_radius_m:=15.0 \
  loop_registration_mode:=std_gicp use_sim_time:=true rviz:=true

# STD retrieval + GICP.
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=std loop_registration_mode:=std_gicp \
  use_sim_time:=true rviz:=true

# Distance retrieval + multi-frame TEASER++ / GICP.
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=distance loop_candidate_radius_m:=15.0 \
  loop_registration_mode:=std_teaser_gicp use_sim_time:=true rviz:=true

# STD retrieval + multi-frame TEASER++ / GICP.
ros2 launch super_lio_slam unitree_slam.launch.py \
  loop_candidate_mode:=std loop_registration_mode:=std_teaser_gicp \
  use_sim_time:=true rviz:=true
```

The old `std_icp` loop mode is removed; use `std_gicp` for direct refinement.
Both modes call the same GICP solver and apply the same overlap, inlier-MSE,
refinement-limit and geometry-degeneracy gates. Direct GICP aligns the current
frame to a historical local submap. With STD retrieval it tries both the STD
pose proposal and the continuous raw-odometry relative pose, applies identical
quality gates to both, and selects the smaller overlap-weighted nearest-neighbor
error (unmatched points receive the squared quality-radius penalty);
`std_teaser_gicp` adds multi-frame source/target windows and TEASER++ coarse registration.

Shared `registration_*`, `gicp_*` and `quality_*` settings come from
`registration_config`. `gicp_submap_half_width` in the SLAM YAML controls the direct
mode's historical neighborhood. Old `icp_*` fine-registration parameters are
removed. The STD library's `icp_threshold` name is retained for its plane-match
score threshold; it does not invoke an ICP solver.

The two TEASER++ / GICP commands require the dependency installation and an
`-DSUPERLIO_WITH_TEASER=ON` build. Both retrieval modes retain the strict sensor-time
gap greater than 30 seconds and the accepted-loop keyframe cooldown. Distance
retrieval skips STD extraction; spatial proximity still requires registration
verification before a loop factor is inserted.

For front-end-only mapping:

```bash
ros2 launch super_lio_slam unitree_slam.launch.py \
  enable_loop_backend:=false use_sim_time:=true rviz:=true
```

This disables both the loop node and keyframe publication, and selects the front-end
RViz view. The `/lio_slam/keyframe` + `/lio_slam/path` export workflow requires the
backend to be enabled.

For VLP32 / UrbanNav, replace `unitree_slam.launch.py` with `vlp32.launch.py` in
any combination. Livox MID-360 uses `Livox_mid360.launch.py`; Robosense Airy uses
`robosense_airy.launch.py`. To select the optional distance-registration trial
profile explicitly:

```bash
ros2 launch super_lio_slam vlp32.launch.py \
  loop_candidate_mode:=distance loop_candidate_radius_m:=15.0 \
  loop_registration_mode:=std_teaser_gicp \
  registration_config:=/home/ros/rosws/SuperLIO_GLIM_ROS2/src/SuperLIO_SLAM/config/loop_distance_teaser_gicp.yaml \
  use_sim_time:=true rviz:=true
```

The default profile remains `loop_teaser_gicp.yaml`. The current trial file uses
coarse RMSE limit 0.4 m and at least 30 coarse inliers. Both profiles share the
same strict GICP gates: source/current overlap >0.8 and inlier MSE <0.1 m². The
coarse settings differ from the default and are not an accuracy guarantee. See the UrbanNav data limitations below.
For bag playback use `ros2 bag play ... --clock` in another sourced terminal;
for live sensors use `use_sim_time:=false`.

Chinese commands and the recording/export sequence are in the
[workspace README](../../README.md#11-不同回环方式的启动命令).

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

Spatial-proximity retrieval remains available for troubleshooting:
`loop_candidate_mode:=distance loop_candidate_radius_m:=15.0`. It searches raw LIO
positions, keeps the strict >30 sensor-second separation, skips STD descriptors,
and passes the raw relative pose as the registration prior. The same TEASER/GICP
quality gates still apply. Use `loop_candidate_mode:=std` for the default STD retrieval.
Both arguments are inherited by sensor launch profiles. The default is STD retrieval
followed by the same raw-odometry radius gate, then TEASER++ and GICP. Distance alone never
creates a loop edge; large odometry drift beyond the radius can hide true revisits.
