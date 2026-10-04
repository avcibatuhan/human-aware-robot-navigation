#ifndef SOCIAL_COSTMAP_LAYER__SOCIAL_COST_HPP_
#define SOCIAL_COSTMAP_LAYER__SOCIAL_COST_HPP_

namespace social_costmap_layer {

struct SocialCostParams {
  double lethal_radius{0.25};  // m: inside it the cost is peak_cost
  double social_radius{1.2};   // m: beyond it no cost is added
  double sigma{0.45};          // m: width of the Gaussian falloff
  unsigned char peak_cost{220};
};

/// Cost added to a cell at `distance` metres from a person:
///   distance <= lethal_radius                -> peak_cost
///   lethal_radius < distance <= social_radius -> peak_cost * exp(-(d - lethal_radius)^2 /
///                                                                (2 sigma^2))
///   distance > social_radius                  -> 0
unsigned char socialCost(double distance, const SocialCostParams & params);

/// The layer never lowers a cell: the result is the larger of the two costs.
unsigned char combineCost(unsigned char existing, unsigned char social);

}  // namespace social_costmap_layer

#endif  // SOCIAL_COSTMAP_LAYER__SOCIAL_COST_HPP_
