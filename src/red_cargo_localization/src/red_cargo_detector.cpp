#include "red_cargo_localization/red_cargo_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace red_cargo_localization
{
namespace
{

struct CellKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const CellKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct CellKeyHash
{
  std::size_t operator()(const CellKey & key) const
  {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct Voxel
{
  double x{};
  double y{};
  double z{};
  std::vector<std::size_t> point_indices;
};

CellKey keyFor(double x, double y, double z, double cell_size)
{
  return {
    static_cast<std::int64_t>(std::floor(x / cell_size)),
    static_cast<std::int64_t>(std::floor(y / cell_size)),
    static_cast<std::int64_t>(std::floor(z / cell_size))};
}

bool finiteAndInRoi(const PointXYZRGB & point, const DetectorParameters & p)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         point.z >= p.min_depth && point.z <= p.max_depth &&
         point.x >= p.min_x && point.x <= p.max_x &&
         point.y >= p.min_y && point.y <= p.max_y;
}

void validate(const DetectorParameters & p)
{
  if (p.min_depth < 0.0 || p.min_depth >= p.max_depth || p.min_x >= p.max_x ||
    p.min_y >= p.max_y || p.voxel_size <= 0.0 || p.cluster_tolerance <= 0.0 ||
    p.voxel_size > p.cluster_tolerance ||
    p.min_cluster_points == 0U || p.min_saturation < 0.0 || p.min_saturation > 1.0 ||
    p.min_value < 0.0 || p.min_value > 1.0 || p.red_hue_low_1 < 0.0 ||
    p.red_hue_high_1 > 360.0 || p.red_hue_low_1 > p.red_hue_high_1 ||
    p.red_hue_low_2 < 0.0 || p.red_hue_high_2 > 360.0 ||
    p.red_hue_low_2 > p.red_hue_high_2)
  {
    throw std::invalid_argument("Invalid red cargo detector parameters");
  }
}

}  // namespace

RedCargoDetector::RedCargoDetector(DetectorParameters parameters)
: parameters_(std::move(parameters))
{
  validate(parameters_);
}

bool RedCargoDetector::isRed(
  std::uint8_t r, std::uint8_t g, std::uint8_t b, const DetectorParameters & p)
{
  const double red = static_cast<double>(r) / 255.0;
  const double green = static_cast<double>(g) / 255.0;
  const double blue = static_cast<double>(b) / 255.0;
  const double maximum = std::max({red, green, blue});
  const double minimum = std::min({red, green, blue});
  const double delta = maximum - minimum;
  const double saturation = maximum == 0.0 ? 0.0 : delta / maximum;

  double hue = 0.0;
  if (delta > std::numeric_limits<double>::epsilon()) {
    if (maximum == red) {
      hue = 60.0 * std::fmod((green - blue) / delta, 6.0);
    } else if (maximum == green) {
      hue = 60.0 * (((blue - red) / delta) + 2.0);
    } else {
      hue = 60.0 * (((red - green) / delta) + 4.0);
    }
    if (hue < 0.0) {
      hue += 360.0;
    }
  }

  const bool hue_in_first = hue >= p.red_hue_low_1 && hue <= p.red_hue_high_1;
  const bool hue_in_second = hue >= p.red_hue_low_2 && hue <= p.red_hue_high_2;
  return (hue_in_first || hue_in_second) && saturation >= p.min_saturation &&
         maximum >= p.min_value;
}

DetectionResult RedCargoDetector::detect(const std::vector<PointXYZRGB> & points) const
{
  std::vector<PointXYZRGB> red_points;
  red_points.reserve(points.size() / 8U);
  for (const auto & point : points) {
    if (finiteAndInRoi(point, parameters_) &&
      isRed(point.r, point.g, point.b, parameters_))
    {
      red_points.push_back(point);
    }
  }

  if (red_points.size() < parameters_.min_cluster_points) {
    return {};
  }

  std::unordered_map<CellKey, std::size_t, CellKeyHash> voxel_lookup;
  std::vector<Voxel> voxels;
  for (std::size_t i = 0; i < red_points.size(); ++i) {
    const auto & point = red_points[i];
    const auto key = keyFor(point.x, point.y, point.z, parameters_.voxel_size);
    auto [iterator, inserted] = voxel_lookup.emplace(key, voxels.size());
    if (inserted) {
      voxels.emplace_back();
    }
    auto & voxel = voxels[iterator->second];
    voxel.x += point.x;
    voxel.y += point.y;
    voxel.z += point.z;
    voxel.point_indices.push_back(i);
  }
  for (auto & voxel : voxels) {
    const auto count = static_cast<double>(voxel.point_indices.size());
    voxel.x /= count;
    voxel.y /= count;
    voxel.z /= count;
  }

  std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> search_grid;
  for (std::size_t i = 0; i < voxels.size(); ++i) {
    search_grid[keyFor(voxels[i].x, voxels[i].y, voxels[i].z, parameters_.cluster_tolerance)]
      .push_back(i);
  }

  const double tolerance_squared = parameters_.cluster_tolerance * parameters_.cluster_tolerance;
  std::vector<bool> visited(voxels.size(), false);
  std::vector<std::size_t> largest_point_indices;
  for (std::size_t start = 0; start < voxels.size(); ++start) {
    if (visited[start]) {
      continue;
    }
    visited[start] = true;
    std::queue<std::size_t> pending;
    pending.push(start);
    std::vector<std::size_t> cluster_points;

    while (!pending.empty()) {
      const auto current_index = pending.front();
      pending.pop();
      const auto & current = voxels[current_index];
      cluster_points.insert(
        cluster_points.end(), current.point_indices.begin(), current.point_indices.end());
      const auto center_key =
        keyFor(current.x, current.y, current.z, parameters_.cluster_tolerance);

      for (std::int64_t dx = -1; dx <= 1; ++dx) {
        for (std::int64_t dy = -1; dy <= 1; ++dy) {
          for (std::int64_t dz = -1; dz <= 1; ++dz) {
            const auto found = search_grid.find(
              {center_key.x + dx, center_key.y + dy, center_key.z + dz});
            if (found == search_grid.end()) {
              continue;
            }
            for (const auto candidate_index : found->second) {
              if (visited[candidate_index]) {
                continue;
              }
              const auto & candidate = voxels[candidate_index];
              const double x_diff = current.x - candidate.x;
              const double y_diff = current.y - candidate.y;
              const double z_diff = current.z - candidate.z;
              if (x_diff * x_diff + y_diff * y_diff + z_diff * z_diff <= tolerance_squared) {
                visited[candidate_index] = true;
                pending.push(candidate_index);
              }
            }
          }
        }
      }
    }
    if (cluster_points.size() > largest_point_indices.size()) {
      largest_point_indices = std::move(cluster_points);
    }
  }

  if (largest_point_indices.size() < parameters_.min_cluster_points) {
    return {};
  }

  DetectionResult result;
  result.detected = true;
  result.points.reserve(largest_point_indices.size());
  for (const auto index : largest_point_indices) {
    const auto & point = red_points[index];
    result.x += point.x;
    result.y += point.y;
    result.z += point.z;
    result.points.push_back(point);
  }
  const auto count = static_cast<double>(result.points.size());
  result.x /= count;
  result.y /= count;
  result.z /= count;
  return result;
}

}  // namespace red_cargo_localization
