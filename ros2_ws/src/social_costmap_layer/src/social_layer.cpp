#include "social_costmap_layer/social_layer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace social_costmap_layer {

void SocialLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SocialLayer: failed to lock the node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("humans_topic", rclcpp::ParameterValue(std::string("/tracked_humans_map")));
  declareParameter("social_radius", rclcpp::ParameterValue(1.2));
  declareParameter("lethal_radius", rclcpp::ParameterValue(0.25));
  declareParameter("sigma", rclcpp::ParameterValue(0.45));
  declareParameter("max_human_age", rclcpp::ParameterValue(2.0));
  declareParameter("peak_cost", rclcpp::ParameterValue(220));

  int peak_cost = 220;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".humans_topic", humans_topic_);
  node->get_parameter(name_ + ".social_radius", params_.social_radius);
  node->get_parameter(name_ + ".lethal_radius", params_.lethal_radius);
  node->get_parameter(name_ + ".sigma", params_.sigma);
  node->get_parameter(name_ + ".max_human_age", max_human_age_);
  node->get_parameter(name_ + ".peak_cost", peak_cost);
  params_.peak_cost = static_cast<unsigned char>(std::clamp(peak_cost, 0, 254));

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<human_interfaces::msg::TrackedHumanArray>(
      humans_topic_, rclcpp::QoS(10),
      std::bind(&SocialLayer::humansCallback, this, std::placeholders::_1), options);

  current_ = true;
  RCLCPP_INFO(logger_,
              "SocialLayer '%s': topic %s, social_radius %.2f, lethal_radius %.2f, sigma %.2f, "
              "max_human_age %.1f, peak_cost %d",
              name_.c_str(), humans_topic_.c_str(), params_.social_radius, params_.lethal_radius,
              params_.sigma, max_human_age_, params_.peak_cost);
}

void SocialLayer::humansCallback(human_interfaces::msg::TrackedHumanArray::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = msg;
}

void SocialLayer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
  humans_.clear();
  current_ = true;
}

void SocialLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                               double * min_x, double * min_y, double * max_x, double * max_y) {
  humans_.clear();

  // Always re-touch the area written last cycle so stale cost is cleared.
  if (has_last_bounds_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
    has_last_bounds_ = false;
  }
  if (!enabled_) {
    return;
  }

  human_interfaces::msg::TrackedHumanArray::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg = latest_;
  }
  if (!msg || msg->humans.empty()) {
    return;
  }

  // Ignore humans whose last update is older than max_human_age.
  const double age =
      (clock_->now() - rclcpp::Time(msg->header.stamp, clock_->get_clock_type())).seconds();
  if (age > max_human_age_) {
    return;
  }

  // The humans arrive in the map frame; a local costmap usually lives in odom.
  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped transform;
  const bool needs_transform = msg->header.frame_id != global_frame;
  if (needs_transform) {
    try {
      transform = tf_->lookupTransform(global_frame, msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "SocialLayer: no transform %s -> %s: %s",
                           msg->header.frame_id.c_str(), global_frame.c_str(), error.what());
      return;
    }
  }

  double human_min_x = 0.0, human_min_y = 0.0, human_max_x = 0.0, human_max_y = 0.0;
  for (const auto & human : msg->humans) {
    geometry_msgs::msg::Point position = human.position;
    if (needs_transform) {
      tf2::doTransform(human.position, position, transform);
    }
    if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
      continue;
    }
    if (humans_.empty()) {
      human_min_x = human_max_x = position.x;
      human_min_y = human_max_y = position.y;
    } else {
      human_min_x = std::min(human_min_x, position.x);
      human_max_x = std::max(human_max_x, position.x);
      human_min_y = std::min(human_min_y, position.y);
      human_max_y = std::max(human_max_y, position.y);
    }
    humans_.push_back(position);
  }
  if (humans_.empty()) {
    return;
  }

  // Only the area the humans affect.
  last_min_x_ = human_min_x - params_.social_radius;
  last_min_y_ = human_min_y - params_.social_radius;
  last_max_x_ = human_max_x + params_.social_radius;
  last_max_y_ = human_max_y + params_.social_radius;
  has_last_bounds_ = true;

  *min_x = std::min(*min_x, last_min_x_);
  *min_y = std::min(*min_y, last_min_y_);
  *max_x = std::max(*max_x, last_max_x_);
  *max_y = std::max(*max_y, last_max_y_);
}

void SocialLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
                              int max_i, int max_j) {
  if (!enabled_ || humans_.empty()) {
    return;
  }
  min_i = std::max(min_i, 0);
  min_j = std::max(min_j, 0);
  max_i = std::min(max_i, static_cast<int>(master_grid.getSizeInCellsX()));
  max_j = std::min(max_j, static_cast<int>(master_grid.getSizeInCellsY()));
  const double resolution = master_grid.getResolution();
  const int reach = static_cast<int>(std::ceil(params_.social_radius / resolution)) + 1;

  for (const auto & human : humans_) {
    // Cell window around this person, clipped to the update window.
    int center_i = 0, center_j = 0;
    master_grid.worldToMapNoBounds(human.x, human.y, center_i, center_j);
    const int from_i = std::max(min_i, center_i - reach);
    const int to_i = std::min(max_i, center_i + reach + 1);
    const int from_j = std::max(min_j, center_j - reach);
    const int to_j = std::min(max_j, center_j + reach + 1);

    for (int j = from_j; j < to_j; ++j) {
      for (int i = from_i; i < to_i; ++i) {
        double wx = 0.0, wy = 0.0;
        master_grid.mapToWorld(i, j, wx, wy);
        const unsigned char social = socialCost(std::hypot(wx - human.x, wy - human.y), params_);
        if (social == 0) {
          continue;
        }
        const unsigned char existing = master_grid.getCost(i, j);
        // Unknown cells stay unknown; everything else is only ever raised.
        if (existing == nav2_costmap_2d::NO_INFORMATION) {
          continue;
        }
        master_grid.setCost(i, j, combineCost(existing, social));
      }
    }
  }
}

}  // namespace social_costmap_layer

PLUGINLIB_EXPORT_CLASS(social_costmap_layer::SocialLayer, nav2_costmap_2d::Layer)
