#include "Scancontext.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <functional>

class ScancontextExtractNode : public rclcpp::Node {
public:
  ScancontextExtractNode() : Node("scancontext_extract_node") {
    const auto topic = declare_parameter<std::string>("input_topic", "/points_raw");
    output_dir_ = declare_parameter<std::string>("output_dir", "/tmp/scancontext_descriptors");
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        "/scancontext/image", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&ScancontextExtractNode::onCloud, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Scan Context extraction: %s -> %s", topic.c_str(), output_dir_.c_str());
  }
private:
  void publishVisualization(const std_msgs::msg::Header &header,
                            const Eigen::MatrixXd &descriptor) {
    constexpr unsigned int scale = 10;
    sensor_msgs::msg::Image image;
    image.header = header;
    image.encoding = "mono8";
    image.height = static_cast<unsigned int>(descriptor.rows()) * scale;
    image.width = static_cast<unsigned int>(descriptor.cols()) * scale;
    image.step = image.width;
    image.data.resize(static_cast<size_t>(image.step) * image.height);
    const double maximum = std::max(0.0, descriptor.maxCoeff());
    for (unsigned int row = 0; row < image.height; ++row) {
      for (unsigned int col = 0; col < image.width; ++col) {
        const double value = descriptor(row / scale, col / scale);
        const double fraction = maximum > 0.0 ?
            std::clamp(value / maximum, 0.0, 1.0) : 0.0;
        image.data[static_cast<size_t>(row) * image.step + col] =
            static_cast<uint8_t>(std::lround(fraction * 255.0));
      }
    }
    image_publisher_->publish(image);
  }
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<SCPointType> cloud;
    try {
      pcl::fromROSMsg(*msg, cloud);
      if (cloud.empty()) return;
      Eigen::MatrixXd descriptor = manager_.makeScancontext(cloud);
      publishVisualization(msg->header, descriptor);
      std::filesystem::create_directories(output_dir_);
      const auto path = std::filesystem::path(output_dir_) /
          ("scancontext_" + std::to_string(sequence_++) + ".csv");
      std::ofstream file(path);
      if (!file) throw std::runtime_error("cannot open output file");
      file << std::setprecision(17);
      for (Eigen::Index row = 0; row < descriptor.rows(); ++row) {
        for (Eigen::Index col = 0; col < descriptor.cols(); ++col) {
          if (col) file << ',';
          file << descriptor(row, col);
        }
        file << '\n';
      }
      RCLCPP_INFO(get_logger(), "Wrote %ldx%ld Scan Context: %s",
                  descriptor.rows(), descriptor.cols(), path.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Scan Context extraction failed: %s", e.what());
    }
  }
  SCManager manager_;
  std::string output_dir_;
  size_t sequence_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScancontextExtractNode>());
  rclcpp::shutdown();
  return 0;
}
