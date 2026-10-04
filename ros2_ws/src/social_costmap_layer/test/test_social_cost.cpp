#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "social_costmap_layer/social_cost.hpp"

using social_costmap_layer::combineCost;
using social_costmap_layer::socialCost;
using social_costmap_layer::SocialCostParams;

namespace {
// Dissertation parameters: lethal 0.25 m, social 1.2 m, sigma 0.45, peak 220.
const SocialCostParams kParams{};

int expected(double distance) {
  const double offset = distance - kParams.lethal_radius;
  return static_cast<int>(std::lround(
      kParams.peak_cost * std::exp(-(offset * offset) / (2 * kParams.sigma * kParams.sigma))));
}
}  // namespace

TEST(SocialCost, DefaultsAreTheDissertationParameters) {
  EXPECT_DOUBLE_EQ(kParams.lethal_radius, 0.25);
  EXPECT_DOUBLE_EQ(kParams.social_radius, 1.2);
  EXPECT_DOUBLE_EQ(kParams.sigma, 0.45);
  EXPECT_EQ(kParams.peak_cost, 220);
}

TEST(SocialCost, PeakCostInsideLethalRadius) {
  EXPECT_EQ(socialCost(0.0, kParams), 220);
  EXPECT_EQ(socialCost(0.1, kParams), 220);
  EXPECT_EQ(socialCost(0.25, kParams), 220);
}

TEST(SocialCost, GaussianFalloffBetweenLethalAndSocialRadius) {
  for (double d : {0.3, 0.5, 0.7, 0.9, 1.1, 1.2}) {
    EXPECT_EQ(socialCost(d, kParams), expected(d)) << "distance " << d;
  }
  // One sigma past the lethal radius: peak * exp(-0.5) = 133.4
  EXPECT_EQ(socialCost(0.25 + 0.45, kParams), 133);
}

TEST(SocialCost, ContinuousAtTheLethalRadius) {
  EXPECT_NEAR(socialCost(0.25 + 1e-6, kParams), 220, 1);
}

TEST(SocialCost, DecreasesMonotonicallyWithDistance) {
  int previous = 255;
  for (double d = 0.0; d <= 1.2; d += 0.01) {
    const int cost = socialCost(d, kParams);
    EXPECT_LE(cost, previous) << "distance " << d;
    previous = cost;
  }
}

TEST(SocialCost, NoCostBeyondSocialRadius) {
  EXPECT_GT(socialCost(1.2, kParams), 0);
  EXPECT_EQ(socialCost(1.2001, kParams), 0);
  EXPECT_EQ(socialCost(5.0, kParams), 0);
}

TEST(SocialCost, NonFiniteDistanceAddsNoCost) {
  EXPECT_EQ(socialCost(std::numeric_limits<double>::quiet_NaN(), kParams), 0);
  EXPECT_EQ(socialCost(std::numeric_limits<double>::infinity(), kParams), 0);
}

TEST(SocialCost, CustomParameters) {
  SocialCostParams params;
  params.lethal_radius = 0.5;
  params.social_radius = 2.0;
  params.sigma = 1.0;
  params.peak_cost = 100;
  EXPECT_EQ(socialCost(0.4, params), 100);
  EXPECT_EQ(socialCost(1.5, params), 61);  // 100 * exp(-0.5)
  EXPECT_EQ(socialCost(2.1, params), 0);
}

TEST(SocialCost, ZeroSigmaOnlyKeepsTheLethalDisc) {
  SocialCostParams params;
  params.sigma = 0.0;
  EXPECT_EQ(socialCost(0.2, params), 220);
  EXPECT_EQ(socialCost(0.5, params), 0);
}

TEST(CombineCost, NeverLowersAnExistingCell) {
  EXPECT_EQ(combineCost(0, 120), 120);
  EXPECT_EQ(combineCost(254, 220), 254);  // a lethal obstacle stays lethal
  EXPECT_EQ(combineCost(253, 220), 253);
  EXPECT_EQ(combineCost(100, 0), 100);
  EXPECT_EQ(combineCost(220, 220), 220);
}
