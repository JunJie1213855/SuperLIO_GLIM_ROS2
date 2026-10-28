# 第三方依赖

TEASER++ 是固定版本的子模块，当前已经通过独立 CMake 安装链接到可选 `std_teaser_gicp` 后端。

- 上游：https://github.com/MIT-SPARK/TEASER-plusplus
- 提交：`52a9c52ee7d4c838c5e8a75458c33178be5bfb70`
- 本目录 `COLCON_IGNORE` 防止 colcon 自动构建上游包。
- 上游源码不修改，FPFH 使用 PCL 实现，不开启上游 `teaser_features`。

在工作区根目录：

```bash
git submodule update --init --recursive
python3 tools/build_teaserpp.py --jobs 1
```

脚本固定 PMC、tinyply、Spectra 提交，保留在 `build/teaserpp_vendor`。缓存齐全后可加 `--offline`，缺少依赖或提交/工作树不符则报错，不覆盖已有修改。构建产物位于 `build/teaserpp`，安装至 `install/teaserpp`，不用 sudo。

主包通过 `find_package(teaserpp CONFIG REQUIRED)` 查找并链接 `teaserpp::teaser_registration`；支持 `teaserpp_DIR` 指定其他兼容安装。只构建旧后端可用 `-DSUPERLIO_WITH_TEASER=OFF`。
