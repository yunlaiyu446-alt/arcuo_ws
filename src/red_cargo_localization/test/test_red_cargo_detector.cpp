#include "red_cargo_localization/red_cargo_detector.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace red_cargo_localization
{
namespace
{

PointXYZRGB point(float x, float y, float z, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  return {x, y, z, r, g, b};
}

DetectorParameters testParameters()
{
  DetectorParameters parameters;
  parameters.min_depth = 0.2;
  parameters.max_depth = 5.0;
  parameters.min_x = -2.0;
  parameters.max_x = 2.0;
  parameters.min_y = -2.0;
  parameters.max_y = 2.0;
  parameters.voxel_size = 0.01;
  parameters.cluster_tolerance = 0.08;
  parameters.min_cluster_points = 3;
  return parameters;
}

TEST(RedCargoDetector, RecognizesBothEndsOfRedHueRange)
{
  const auto parameters = testParameters();
  EXPECT_TRUE(RedCargoDetector::isRed(255, 10, 0, parameters));
  EXPECT_TRUE(RedCargoDetector::isRed(255, 0, 10, parameters));
  EXPECT_FALSE(RedCargoDetector::isRed(0, 255, 0, parameters));
  EXPECT_FALSE(RedCargoDetector::isRed(100, 95, 95, parameters));
  EXPECT_FALSE(RedCargoDetector::isRed(30, 0, 0, parameters));
}

TEST(RedCargoDetector, FiltersInvalidAndOutOfRoiPoints)
{
  auto parameters = testParameters();
  parameters.min_cluster_points = 2;
  RedCargoDetector detector(parameters);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto result = detector.detect({
    point(0.00F, 0.00F, 1.00F, 255, 0, 0),
    point(0.02F, 0.00F, 1.00F, 240, 10, 5),
    point(nan, 0.00F, 1.00F, 255, 0, 0),
    point(0.00F, 0.00F, 0.00F, 255, 0, 0),
    point(3.00F, 0.00F, 1.00F, 255, 0, 0),
    point(0.01F, 0.00F, 1.00F, 0, 255, 0),
  });
  ASSERT_TRUE(result.detected);
  EXPECT_EQ(result.points.size(), 2U);
  EXPECT_NEAR(result.x, 0.01, 1e-6);
  EXPECT_NEAR(result.y, 0.0, 1e-6);
  EXPECT_NEAR(result.z, 1.0, 1e-6);
}

TEST(RedCargoDetector, SelectsLargestClusterAndAveragesOriginalPoints)
{
  RedCargoDetector detector(testParameters());
  const auto result = detector.detect({
    point(0.00F, 0.00F, 1.00F, 255, 0, 0),
    point(0.02F, 0.00F, 1.00F, 255, 5, 0),
    point(0.04F, 0.00F, 1.00F, 250, 0, 5),
    point(0.06F, 0.00F, 1.00F, 250, 5, 5),
    point(1.00F, 1.00F, 2.00F, 255, 0, 0),
    point(1.02F, 1.00F, 2.00F, 255, 0, 0),
    point(1.04F, 1.00F, 2.00F, 255, 0, 0),
    point(-1.0F, -1.0F, 4.0F, 255, 0, 0),
  });
  ASSERT_TRUE(result.detected);
  EXPECT_EQ(result.points.size(), 4U);
  EXPECT_NEAR(result.x, 0.03, 1e-6);
  EXPECT_NEAR(result.y, 0.0, 1e-6);
  EXPECT_NEAR(result.z, 1.0, 1e-6);
}

TEST(RedCargoDetector, ReportsNoDetectionBelowMinimumClusterSize)
{
  RedCargoDetector detector(testParameters());
  const auto result = detector.detect({
    point(0.0F, 0.0F, 1.0F, 255, 0, 0),
    point(0.02F, 0.0F, 1.0F, 255, 0, 0),
    point(1.0F, 1.0F, 1.0F, 255, 0, 0),
  });
  EXPECT_FALSE(result.detected);
  EXPECT_TRUE(result.points.empty());
}

TEST(RedCargoDetector, RejectsInvalidConfiguration)
{
  auto parameters = testParameters();
  parameters.voxel_size = 0.0;
  EXPECT_THROW(RedCargoDetector detector(parameters), std::invalid_argument);

  parameters = testParameters();
  parameters.voxel_size = parameters.cluster_tolerance + 0.01;
  EXPECT_THROW(RedCargoDetector detector(parameters), std::invalid_argument);

  parameters = testParameters();
  parameters.red_hue_high_2 = 361.0;
  EXPECT_THROW(RedCargoDetector detector(parameters), std::invalid_argument);
}

}  // namespace
}  // namespace red_cargo_localization
