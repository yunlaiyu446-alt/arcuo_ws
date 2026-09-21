#include "red_cargo_localization/red_cargo_detector.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace red_cargo_localization
{
namespace
{

const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const auto iterator = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const auto & field) {return field.name == name;});
  return iterator == cloud.fields.end() ? nullptr : &*iterator;
}

std::uint32_t readPackedRgb(
  const std::uint8_t * data, const sensor_msgs::msg::PointField & field)
{
  std::uint32_t packed{};
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32 ||
    field.datatype == sensor_msgs::msg::PointField::UINT32 ||
    field.datatype == sensor_msgs::msg::PointField::INT32)
  {
    std::memcpy(&packed, data + field.offset, sizeof(packed));
    return packed;
  }
  throw std::runtime_error("The rgb/rgba point field must be FLOAT32, UINT32, or INT32");
}

std::uint8_t readColorChannel(
  const std::uint8_t * data, const sensor_msgs::msg::PointField & field)
{
  if (field.datatype != sensor_msgs::msg::PointField::UINT8 &&
    field.datatype != sensor_msgs::msg::PointField::INT8)
  {
    throw std::runtime_error("Separate r/g/b point fields must be UINT8 or INT8");
  }
  return *(data + field.offset);
}

DetectorParameters declareDetectorParameters(rclcpp::Node & node)
{
  DetectorParameters parameters;
  parameters.min_depth = node.declare_parameter("min_depth", parameters.min_depth);
  parameters.max_depth = node.declare_parameter("max_depth", parameters.max_depth);
  parameters.min_x = node.declare_parameter("min_x", parameters.min_x);
  parameters.max_x = node.declare_parameter("max_x", parameters.max_x);
  parameters.min_y = node.declare_parameter("min_y", parameters.min_y);
  parameters.max_y = node.declare_parameter("max_y", parameters.max_y);
  parameters.red_hue_low_1 = node.declare_parameter("red_hue_low_1", parameters.red_hue_low_1);
  parameters.red_hue_high_1 = node.declare_parameter("red_hue_high_1", parameters.red_hue_high_1);
  parameters.red_hue_low_2 = node.declare_parameter("red_hue_low_2", parameters.red_hue_low_2);
  parameters.red_hue_high_2 = node.declare_parameter("red_hue_high_2", parameters.red_hue_high_2);
  parameters.min_saturation = node.declare_parameter("min_saturation", parameters.min_saturation);
  parameters.min_value = node.declare_parameter("min_value", parameters.min_value);
  parameters.voxel_size = node.declare_parameter("voxel_size", parameters.voxel_size);
  parameters.cluster_tolerance =
    node.declare_parameter("cluster_tolerance", parameters.cluster_tolerance);
  const auto min_cluster_points = node.declare_parameter<std::int64_t>(
    "min_cluster_points", static_cast<std::int64_t>(parameters.min_cluster_points));
  if (min_cluster_points <= 0) {
    throw std::invalid_argument("min_cluster_points must be greater than zero");
  }
  parameters.min_cluster_points = static_cast<std::size_t>(min_cluster_points);
  return parameters;
}

}  // namespace

class RedCargoLocalizerNode : public rclcpp::Node
{
public:
  RedCargoLocalizerNode()
  : Node("red_cargo_localizer"), detector_(declareDetectorParameters(*this))
  {
    const auto input_topic = declare_parameter("input_topic", "/camera/depth_registered/points");
    publish_red_cloud_ = declare_parameter("publish_red_cloud", true);

    centroid_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("~/centroid", 10);
    detected_publisher_ = create_publisher<std_msgs::msg::Bool>("~/detected", 10);
    point_count_publisher_ = create_publisher<std_msgs::msg::UInt32>("~/point_count", 10);
    if (publish_red_cloud_) {
      red_cloud_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("~/red_points", rclcpp::SensorDataQoS());
    }
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&RedCargoLocalizerNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening for colored point clouds on %s", input_topic.c_str());
  }

private:
  std::vector<PointXYZRGB> parseCloud(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    if (cloud.is_bigendian) {
      throw std::runtime_error("Big-endian point clouds are not supported");
    }
    const auto * x_field = findField(cloud, "x");
    const auto * y_field = findField(cloud, "y");
    const auto * z_field = findField(cloud, "z");
    if (!x_field || !y_field || !z_field ||
      x_field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      y_field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      z_field->datatype != sensor_msgs::msg::PointField::FLOAT32)
    {
      throw std::runtime_error("Point cloud requires FLOAT32 x, y, and z fields");
    }

    const auto * rgb_field = findField(cloud, "rgb");
    if (!rgb_field) {
      rgb_field = findField(cloud, "rgba");
    }
    const auto * r_field = findField(cloud, "r");
    const auto * g_field = findField(cloud, "g");
    const auto * b_field = findField(cloud, "b");
    if (!rgb_field && (!r_field || !g_field || !b_field)) {
      throw std::runtime_error("Point cloud requires an rgb/rgba field or separate r, g, b fields");
    }

    const auto fields_fit = [&cloud](const sensor_msgs::msg::PointField * field, std::size_t size) {
        return field && static_cast<std::size_t>(field->offset) + size <= cloud.point_step;
      };
    if (!fields_fit(x_field, sizeof(float)) || !fields_fit(y_field, sizeof(float)) ||
      !fields_fit(z_field, sizeof(float)) ||
      (rgb_field && !fields_fit(rgb_field, sizeof(std::uint32_t))) ||
      (!rgb_field &&
      (!fields_fit(r_field, sizeof(std::uint8_t)) || !fields_fit(g_field, sizeof(std::uint8_t)) ||
      !fields_fit(b_field, sizeof(std::uint8_t)))))
    {
      throw std::runtime_error("Point field offset exceeds point_step");
    }

    const auto point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    const auto required_size = cloud.height == 0U ? 0U :
      (static_cast<std::size_t>(cloud.height - 1U) * cloud.row_step) +
      (static_cast<std::size_t>(cloud.width) * cloud.point_step);
    if (cloud.point_step == 0U || cloud.row_step < cloud.width * cloud.point_step ||
      cloud.data.size() < required_size)
    {
      throw std::runtime_error("Point cloud data buffer is smaller than its dimensions");
    }
    std::vector<PointXYZRGB> points;
    points.reserve(point_count);
    for (std::size_t row = 0; row < cloud.height; ++row) {
      for (std::size_t column = 0; column < cloud.width; ++column) {
        const auto * data = cloud.data.data() + row * cloud.row_step + column * cloud.point_step;
        PointXYZRGB point;
        std::memcpy(&point.x, data + x_field->offset, sizeof(float));
        std::memcpy(&point.y, data + y_field->offset, sizeof(float));
        std::memcpy(&point.z, data + z_field->offset, sizeof(float));
        if (rgb_field) {
          const auto packed = readPackedRgb(data, *rgb_field);
          point.r = static_cast<std::uint8_t>((packed >> 16U) & 0xffU);
          point.g = static_cast<std::uint8_t>((packed >> 8U) & 0xffU);
          point.b = static_cast<std::uint8_t>(packed & 0xffU);
        } else {
          point.r = readColorChannel(data, *r_field);
          point.g = readColorChannel(data, *g_field);
          point.b = readColorChannel(data, *b_field);
        }
        points.push_back(point);
      }
    }
    return points;
  }

  sensor_msgs::msg::PointCloud2 makeRedCloud(
    const std_msgs::msg::Header & header, const std::vector<PointXYZRGB> & points) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(points.size());
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> r(cloud, "r");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> g(cloud, "g");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> b(cloud, "b");
    for (const auto & point : points) {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      *r = point.r;
      *g = point.g;
      *b = point.b;
      ++x;
      ++y;
      ++z;
      ++r;
      ++g;
      ++b;
    }
    return cloud;
  }

  void publishStatus(bool detected, std::size_t point_count)
  {
    std_msgs::msg::Bool detected_message;
    detected_message.data = detected;
    detected_publisher_->publish(detected_message);
    std_msgs::msg::UInt32 count_message;
    count_message.data = static_cast<std::uint32_t>(
      std::min(point_count, static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    point_count_publisher_->publish(count_message);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    try {
      const auto result = detector_.detect(parseCloud(*cloud));
      publishStatus(result.detected, result.points.size());
      if (!result.detected) {
        return;
      }

      geometry_msgs::msg::PointStamped centroid;
      centroid.header = cloud->header;
      centroid.point.x = result.x;
      centroid.point.y = result.y;
      centroid.point.z = result.z;
      centroid_publisher_->publish(centroid);
      if (red_cloud_publisher_) {
        red_cloud_publisher_->publish(makeRedCloud(cloud->header, result.points));
      }
    } catch (const std::exception & error) {
      publishStatus(false, 0U);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to process colored point cloud: %s",
        error.what());
    }
  }

  RedCargoDetector detector_;
  bool publish_red_cloud_{true};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr point_count_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr red_cloud_publisher_;
};

}  // namespace red_cargo_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<red_cargo_localization::RedCargoLocalizerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("red_cargo_localizer"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
