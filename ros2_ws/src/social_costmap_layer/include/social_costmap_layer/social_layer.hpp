#ifndef SOCIAL_COSTMAP_LAYER__SOCIAL_LAYER_HPP_
#define SOCIAL_COSTMAP_LAYER__SOCIAL_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "human_interfaces/msg/tracked_human_array.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "social_costmap_layer/social_cost.hpp"

namespace social_costmap_layer {

/// Costmap layer that adds a Gaussian-like cost around each tracked human.
class SocialLayer : public nav2_costmap_2d::Layer {
public:
  SocialLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x,
                    double * min_y, double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
                   int max_j) override;
  void reset() override;
  bool isClearable() override { return false; }

private:
  void humansCallback(human_interfaces::msg::TrackedHumanArray::ConstSharedPtr msg);

  rclcpp::Subscription<human_interfaces::msg::TrackedHumanArray>::SharedPtr subscription_;
  std::mutex mutex_;
  human_interfaces::msg::TrackedHumanArray::ConstSharedPtr latest_;

  // Human positions in the costmap's global frame, refreshed in updateBounds.
  std::vector<geometry_msgs::msg::Point> humans_;

  // Area written in the previous cycle; it is included in the next bounds so
  // the cost left behind by a person who moved or disappeared is cleared.
  bool has_last_bounds_{false};
  double last_min_x_{0.0}, last_min_y_{0.0}, last_max_x_{0.0}, last_max_y_{0.0};

  SocialCostParams params_;
  double max_human_age_{2.0};
  std::string humans_topic_;
};

}  // namespace social_costmap_layer

#endif  // SOCIAL_COSTMAP_LAYER__SOCIAL_LAYER_HPP_
