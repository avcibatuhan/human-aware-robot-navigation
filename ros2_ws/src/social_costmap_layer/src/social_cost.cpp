#include "social_costmap_layer/social_cost.hpp"

#include <algorithm>
#include <cmath>

namespace social_costmap_layer {

unsigned char socialCost(double distance, const SocialCostParams & params) {
  if (!std::isfinite(distance) || distance > params.social_radius) {
    return 0;
  }
  if (distance <= params.lethal_radius) {
    return params.peak_cost;
  }
  if (params.sigma <= 0.0) {
    return 0;
  }
  const double offset = distance - params.lethal_radius;
  const double cost =
      params.peak_cost * std::exp(-(offset * offset) / (2.0 * params.sigma * params.sigma));
  return static_cast<unsigned char>(std::lround(std::clamp(cost, 0.0, 255.0)));
}

unsigned char combineCost(unsigned char existing, unsigned char social) {
  return std::max(existing, social);
}

}  // namespace social_costmap_layer
