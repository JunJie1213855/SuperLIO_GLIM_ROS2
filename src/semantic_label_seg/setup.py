from glob import glob

from setuptools import find_packages, setup

setup(
    name="semantic_label_seg",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/semantic_label_seg"]),
        (
            "share/semantic_label_seg",
            ["package.xml", "README.md", "LICENSE", "THIRD_PARTY.md"],
        ),
        ("share/semantic_label_seg/config", glob("config/*.yaml")),
        ("share/semantic_label_seg/launch", glob("launch/*.launch.py")),
        ("share/semantic_label_seg/third_party", glob("third_party/*")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    extras_require={"lidar": ["torch>=2.0"], "image": ["ultralytics"], "test": ["pytest"]},
    zip_safe=False,
    maintainer="ROS workspace maintainers",
    maintainer_email="maintainer@example.com",
    description="Standalone SLIDE-SLAM semantic segmentation library and ROS 2 interfaces",
    license="BSD-3-Clause AND MIT",
    entry_points={
        "console_scripts": [
            "lidar_seg_node = semantic_label_seg.lidar_node:main",
            "rgbd_seg_node = semantic_label_seg.rgbd_node:main",
        ]
    },
)
