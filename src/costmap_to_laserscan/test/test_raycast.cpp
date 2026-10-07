// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "costmap_to_laserscan/costmap_to_laserscan_node.hpp"

using costmap_to_laserscan::BlockedLut;
using costmap_to_laserscan::GridView;
using costmap_to_laserscan::RayCastConfig;
using costmap_to_laserscan::castScan;
using costmap_to_laserscan::costToOccupancy;

namespace
{

constexpr uint32_t kW = 20;
constexpr uint32_t kH = 20;
constexpr float kRes = 0.1F;
constexpr float kInf = std::numeric_limits<float>::infinity();

struct Fixture
{
  std::vector<uint8_t> cells{std::vector<uint8_t>(kW * kH, 0U)};
  BlockedLut lut{};

  Fixture()
  {
    lut.fill(0U);
    lut[100] = 1U;
  }

  void block(uint32_t x, uint32_t y) {cells[y * kW + x] = 100U;}

  GridView view() const {return GridView{cells.data(), kW, kH};}
};

RayCastConfig baseConfig()
{
  RayCastConfig cfg;
  cfg.origin_x_cells = 10.0F;
  cfg.origin_y_cells = 10.5F;
  cfg.base_cos = 1.0F;
  cfg.base_sin = 0.0F;
  cfg.t_min_cells = 0.0F;
  cfg.t_max_cells = 100.0F;
  cfg.resolution = kRes;
  cfg.no_return = kInf;
  return cfg;
}

float castSingle(const Fixture & f, const RayCastConfig & cfg, double angle)
{
  const float c = static_cast<float>(std::cos(angle));
  const float s = static_cast<float>(std::sin(angle));
  float range = 0.0F;
  castScan(f.view(), f.lut, cfg, &c, &s, &range, 1U);
  return range;
}

}  // namespace

TEST(RayCast, EmptyGridReturnsNoReturn)
{
  Fixture f;
  EXPECT_EQ(castSingle(f, baseConfig(), 0.0), kInf);
}

TEST(RayCast, AxisAlignedHitIsExact)
{
  Fixture f;
  f.block(15, 10);
  // Beam enters column 15 exactly 5 cells away.
  EXPECT_NEAR(castSingle(f, baseConfig(), 0.0), 0.5F, 1e-4F);
}

TEST(RayCast, DiagonalHitUsesCellEntryDistance)
{
  Fixture f;
  f.block(12, 12);
  RayCastConfig cfg = baseConfig();
  cfg.origin_x_cells = 10.0F;
  cfg.origin_y_cells = 10.0F;
  EXPECT_NEAR(castSingle(f, cfg, M_PI / 4.0), 2.0F * std::sqrt(2.0F) * kRes, 1e-4F);
}

TEST(RayCast, NearestObstacleWins)
{
  Fixture f;
  f.block(12, 10);
  f.block(15, 10);
  EXPECT_NEAR(castSingle(f, baseConfig(), 0.0), 0.2F, 1e-4F);
}

TEST(RayCast, ObstaclesInsideMinRangeAreSkipped)
{
  Fixture f;
  f.block(11, 10);   // 0.1 m - inside the footprint
  f.block(15, 10);
  RayCastConfig cfg = baseConfig();
  cfg.t_min_cells = 2.0F;   // range_min = 0.2 m
  EXPECT_NEAR(castSingle(f, cfg, 0.0), 0.5F, 1e-4F);
}

TEST(RayCast, MaxRangeTruncates)
{
  Fixture f;
  f.block(15, 10);
  RayCastConfig cfg = baseConfig();
  cfg.t_max_cells = 3.0F;   // 0.3 m
  EXPECT_EQ(castSingle(f, cfg, 0.0), kInf);
}

TEST(RayCast, SensorOutsideRasterIsClipped)
{
  Fixture f;
  f.block(3, 10);
  RayCastConfig cfg = baseConfig();
  cfg.origin_x_cells = -5.0F;
  EXPECT_NEAR(castSingle(f, cfg, 0.0), 0.8F, 1e-4F);
}

TEST(RayCast, BeamPointingAwayFromRasterMisses)
{
  Fixture f;
  f.block(3, 10);
  RayCastConfig cfg = baseConfig();
  cfg.origin_x_cells = -5.0F;
  EXPECT_EQ(castSingle(f, cfg, M_PI), kInf);
}

TEST(RayCast, FrameRotationIsApplied)
{
  Fixture f;
  f.block(10, 15);
  RayCastConfig cfg = baseConfig();
  cfg.origin_x_cells = 10.5F;
  cfg.origin_y_cells = 10.0F;
  cfg.base_cos = 0.0F;   // scan frame yawed +90 deg inside the grid frame
  cfg.base_sin = 1.0F;
  EXPECT_NEAR(castSingle(f, cfg, 0.0), 0.5F, 1e-4F);
}

TEST(RayCast, FullCircleIsConsistent)
{
  Fixture f;
  for (uint32_t x = 0; x < kW; ++x) {
    f.block(x, 0);
    f.block(x, kH - 1);
  }
  for (uint32_t y = 0; y < kH; ++y) {
    f.block(0, y);
    f.block(kW - 1, y);
  }
  constexpr std::size_t kBeams = 720;
  std::vector<float> cos_t(kBeams);
  std::vector<float> sin_t(kBeams);
  for (std::size_t i = 0; i < kBeams; ++i) {
    const double a = -M_PI + static_cast<double>(i) * (2.0 * M_PI / kBeams);
    cos_t[i] = static_cast<float>(std::cos(a));
    sin_t[i] = static_cast<float>(std::sin(a));
  }
  RayCastConfig cfg = baseConfig();
  cfg.origin_x_cells = 10.0F;
  cfg.origin_y_cells = 10.0F;

  std::vector<float> ranges(kBeams, 0.0F);
  castScan(f.view(), f.lut, cfg, cos_t.data(), sin_t.data(), ranges.data(), kBeams);

  // A closed box: every beam must terminate, and never beyond the box diagonal.
  for (std::size_t i = 0; i < kBeams; ++i) {
    ASSERT_TRUE(std::isfinite(ranges[i])) << "beam " << i;
    EXPECT_GE(ranges[i], 0.9F * 9.0F * kRes);
    EXPECT_LE(ranges[i], std::sqrt(2.0F) * 10.0F * kRes + 1e-3F);
  }
}

TEST(CostTranslation, MatchesNav2Table)
{
  EXPECT_EQ(costToOccupancy(0), 0);
  EXPECT_EQ(costToOccupancy(253), 99);
  EXPECT_EQ(costToOccupancy(254), 100);
  EXPECT_EQ(costToOccupancy(252), 98);
  EXPECT_GT(costToOccupancy(128), 0);
  EXPECT_LT(costToOccupancy(128), 99);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}