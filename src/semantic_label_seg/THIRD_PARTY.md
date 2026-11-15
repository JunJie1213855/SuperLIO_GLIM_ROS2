# Source provenance

This package extracts the semantic-labeling portion of KumarRobotics SLIDE-SLAM,
from local commit `63c78b9995b1e068dc0f185b3550e837b524833c` (verify against
the source repository history if importing another revision).

- Source repository: https://github.com/KumarRobotics/SLIDE_SLAM
- LiDAR frontend: `frontend/scan2shape/script/infer_node.py`, `laserscan.py`,
  `segmentator.py`, `load_model.py`, `CRF.py`, `backbone/`, and `decoder/`.
- Image frontend: `frontend/object_modeller/object_detector_utils/detect.py`.
- The architecture implementations in `semantic_label_seg/models/backbone/`
  and `decoder/` are copied from that frontend, preserving their upstream source
  comments. `models/crf.py` is copied from `CRF.py`.
- The projection, inference interface, image-label composition, and ROS 2
  interfaces are refactored adaptations; they do not require SLIDE-SLAM.

SLIDE-SLAM's BSD-3-Clause notice is retained in `LICENSE` and
`third_party/SLIDE_SLAM_LICENSE.txt`. Its RangeNet frontend identifies PRBonn
lidar-bonnetal as its upstream source; that project's MIT notice is retained in
`third_party/lidar-bonnetal_LICENSE.txt`. Backbone/decoder files also retain
their attribution to https://github.com/BobLiu20/YOLOv3_PyTorch.

Pretrained RangeNet and YOLO checkpoints are not included. External runtime
packages (including PyTorch and Ultralytics) and model checkpoints retain their
own licenses and usage terms.
