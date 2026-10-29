#include "loop/OptimizedMapWriter.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unistd.h>

namespace super_lio_slam
{
  namespace
  {
    struct Key
    {
      std::array<int64_t, 3> index;
      bool operator==(const Key &other) const { return index == other.index; }
    };
    struct Hash
    {
      std::size_t operator()(const Key &key) const
      {
        std::size_t h = 0;
        for (auto i : key.index)
          h ^= std::hash<int64_t>{}(i) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
      }
    };
    struct Cell
    {
      Eigen::Vector4d sum = Eigen::Vector4d::Zero();
      std::size_t count = 0;
    };
  }

  std::size_t writeOptimizedMap(const std::vector<OptimizedMapFrame> &frames,
                                const std::string &path, double voxel_size)
  {
    if (path.empty() || !std::isfinite(voxel_size) || voxel_size < 0)
      throw std::invalid_argument("save path must be nonempty and voxel size finite and nonnegative");
    const auto output = std::filesystem::absolute(path);
    std::filesystem::create_directories(output.parent_path());
    std::string pattern = output.string() + ".tmp-XXXXXX";
    const int fd = mkstemp(pattern.data());
    if (fd < 0)
      throw std::runtime_error("Cannot create temporary PCD beside " + output.string());
    close(fd);
    try
    {
      std::ofstream stream(pattern, std::ios::binary | std::ios::trunc);
      stream.exceptions(std::ios::failbit | std::ios::badbit);
      stream << "# Optimized map: frame_id=map\nVERSION 0.7\nFIELDS x y z intensity\n"
                "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH ";
      const auto width_offset = stream.tellp();
      stream << std::setw(20) << 0 << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ";
      const auto count_offset = stream.tellp();
      stream << std::setw(20) << 0 << "\nDATA binary\n";
      std::size_t count = 0;
      auto write = [&](const Eigen::Vector4d &p)
      {
        const std::array<float, 4> values{static_cast<float>(p.x()), static_cast<float>(p.y()),
                                          static_cast<float>(p.z()), static_cast<float>(p.w())};
        stream.write(reinterpret_cast<const char *>(values.data()), sizeof(values));
        ++count;
      };
      std::unordered_map<Key, Cell, Hash> voxels;
      for (const auto &frame : frames)
      {
        if (!frame.pose.matrix().allFinite())
          throw std::runtime_error("Nonfinite optimized pose");
        if (!frame.cloud)
          continue;
        for (const auto &point : *frame.cloud)
        {
          if (!point.getVector3fMap().allFinite())
            continue;
          const Eigen::Vector3d p = frame.pose * point.getVector3fMap().cast<double>();
          if (!p.allFinite() || p.cwiseAbs().maxCoeff() > std::numeric_limits<float>::max())
            throw std::runtime_error("Transformed point outside float PCD range");
          const Eigen::Vector4d value(p.x(), p.y(), p.z(),
                                      std::isfinite(point.intensity) ? point.intensity : 0.0);
          if (voxel_size == 0)
            write(value);
          else
          {
            Key key;
            for (int axis = 0; axis < 3; ++axis)
            {
              const double index = std::floor(p[axis] / voxel_size);
              if (!std::isfinite(index) || index < -9.22e18 || index > 9.22e18)
                throw std::runtime_error("Voxel index outside int64 range");
              key.index[axis] = static_cast<int64_t>(index);
            }
            auto &cell = voxels[key];
            cell.sum += value;
            ++cell.count;
          }
        }
      }
      for (const auto &[key, cell] : voxels)
        write(cell.sum / static_cast<double>(cell.count));
      if (count == 0)
        throw std::runtime_error("No finite points to save");
      stream.seekp(width_offset);
      stream << std::setw(20) << count;
      stream.seekp(count_offset);
      stream << std::setw(20) << count;
      stream.close();
      // Same-filesystem rename keeps the previous complete map on write failure.
      std::filesystem::rename(pattern, output);
      return count;
    }
    catch (...)
    {
      std::error_code ignored;
      std::filesystem::remove(pattern, ignored);
      throw;
    }
  }
}
