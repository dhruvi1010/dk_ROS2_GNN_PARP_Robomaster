#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2_ros/buffer.h"
#include "perception_aware_nav2_msgs/msg/link_stats.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_comms_risk_layer
{

class CommsRiskLayer : public nav2_costmap_2d::Layer
{
public:
  CommsRiskLayer();
  ~CommsRiskLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  bool isClearable() override { return true; }

private:
  void linkStatsCallback(
    const perception_aware_nav2_msgs::msg::LinkStats::SharedPtr msg);

  // Convert a LinkStats reading into a scalar risk in [0, 1].
  double riskFromStats(const perception_aware_nav2_msgs::msg::LinkStats & s) const;

  // Sample storage — a sparse list of (wx, wy, risk, stamp) tuples.
  struct Sample {
    double wx;
    double wy;
    double risk;         // [0, 1]
    rclcpp::Time stamp;
  };

  // EMA-update the sample nearest to (wx, wy) within inflation radius, else append.
  void fuseSample(double wx, double wy, double risk, const rclcpp::Time & t);

  // Subscription + state
  rclcpp::Subscription<perception_aware_nav2_msgs::msg::LinkStats>::SharedPtr sub_;
  std::vector<Sample> samples_;
  std::mutex mutex_;

  // Parameters
  std::string topic_;
  double alpha_{0.2};
  double rsrp_min_dbm_{-85.0};
  double rsrp_range_db_{25.0};
  double jitter_max_ms_{30.0};
  double jitter_range_ms_{40.0};
  double w_rsrp_{0.6};
  double w_jitter_{0.4};
  double w_loss_{0.0};             // optional third term; off by default
  double k_cost_{120.0};           // peak cost written per cell (<= 200)
  double inflation_radius_m_{1.2};
  double decay_time_s_{30.0};
  bool   only_raise_{true};
  std::string global_frame_;       // usually "map"

  // Bounds the last touched region writes into updateBounds().
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool   have_bounds_{false};

  // Most recent robot pose seen by updateBounds(); reused by the async
  // LinkStats callback because nav2_costmap_2d::Layer has no direct
  // getRobotPose() — that method lives on Costmap2DROS.
  double last_robot_x_{0.0};
  double last_robot_y_{0.0};
  bool   have_robot_pose_{false};


};

}  // namespace nav2_comms_risk_layer