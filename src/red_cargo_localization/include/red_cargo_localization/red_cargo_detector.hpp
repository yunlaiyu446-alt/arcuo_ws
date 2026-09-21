#ifndef RED_CARGO_LOCALIZATION__RED_CARGO_DETECTOR_HPP_
#define RED_CARGO_LOCALIZATION__RED_CARGO_DETECTOR_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace red_cargo_localization
{

struct PointXYZRGB
{
  float x{};
  float y{};
  float z{};
  std::uint8_t r{};
  std::uint8_t g{};
  std::uint8_t b{};
};

struct DetectorParameters
{
  double min_depth{0.2};
  double max_depth{10.0};
  double min_x{-100.0};
  double max_x{100.0};
  double min_y{-100.0};
  double max_y{100.0};
  double red_hue_low_1{0.0};
  double red_hue_high_1{15.0};
  double red_hue_low_2{345.0};
  double red_hue_high_2{360.0};
  double min_saturation{0.45};
  double min_value{0.15};
  double voxel_size{0.01};
  double cluster_tolerance{0.05};
  std::size_t min_cluster_points{30};
};

struct DetectionResult
{
  bool detected{false};
  double x{};
  double y{};
  double z{};
  std::vector<PointXYZRGB> points;
};

class RedCargoDetector
{
public:
  explicit RedCargoDetector(DetectorParameters parameters);
  DetectionResult detect(const std::vector<PointXYZRGB> & points) const;
  static bool isRed(std::uint8_t r, std::uint8_t g, std::uint8_t b,
    const DetectorParameters & parameters);

private:
  DetectorParameters parameters_;
};

}  // namespace red_cargo_localization

#endif  // RED_CARGO_LOCALIZATION__RED_CARGO_DETECTOR_HPP_
