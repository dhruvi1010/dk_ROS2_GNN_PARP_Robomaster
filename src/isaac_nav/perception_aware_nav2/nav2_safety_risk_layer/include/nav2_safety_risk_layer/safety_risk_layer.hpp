#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "gnn_interfaces/msg/tracked_polygon.hpp"

namespace nav2_safety_risk_layer
{

class SafetyRiskLayer : public nav2_costmap_2d::Layer
{
public:
  SafetyRiskLayer();
  ~SafetyRiskLayer() override = default;

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
  struct HistoryPoint
  {
    rclcpp::Time stamp;
    double cx;
    double cy;
  };

  struct Track
  {
    uint32_t label{0};
    double cx{0.0};
    double cy{0.0};
    rclcpp::Time last_t;
    std::deque<HistoryPoint> history;
  };

  void polygonCallback(
    const gnn_interfaces::msg::TrackedPolygon::SharedPtr msg);

  bool isDynamicLabel(uint32_t label) const;
  bool estimateVelocity(const Track & tr, double & vx, double & vy) const;
  void prune(const rclcpp::Time & now);

  // --- parameters ----------------------------------------------------------
  std::string topic_;
  std::vector<int64_t> dynamic_labels_;   // ROS int_array → int64_t
  double match_radius_m_{0.4};
  double decay_time_s_{20.0};
  double history_window_s_{2.0};
  size_t max_history_{8};
  double safety_horizon_s_{3.0};
  double safety_dt_s_{0.3};
  double halo_radius_m_{0.6};
  double k_safety_cost_{100.0};
  bool   fade_with_horizon_{true};
  bool   only_raise_{true};

  std::string global_frame_;

  rclcpp::Subscription<gnn_interfaces::msg::TrackedPolygon>::SharedPtr sub_;

  std::vector<Track> tracks_;
  std::mutex mutex_;

  // bounds region for updateBounds()
  double last_min_x_{0}, last_min_y_{0}, last_max_x_{0}, last_max_y_{0};
  bool   have_bounds_{false};
};

}  // namespace nav2_safety_risk_layer