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
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gnn_interfaces/msg/tracked_polygon.hpp"

namespace nav2_o11y_risk_layer
{

class O11yRiskLayer : public nav2_costmap_2d::Layer
{
public:
  O11yRiskLayer();
  ~O11yRiskLayer() override = default;

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
  void polygonCallback(
    const gnn_interfaces::msg::TrackedPolygon::SharedPtr msg);

  // risk = a_uncertainty * (1 - confidence) + b_fragility * max(contributor_ratios)
  double riskFromPolygon(const gnn_interfaces::msg::TrackedPolygon & p) const;

  // Cached, already-transformed polygon (in global frame `map`).
  struct Track {
    double cx;                       // centroid x in map frame
    double cy;                       // centroid y in map frame
    double risk;                     // [0, 1]
    uint32_t label;
    rclcpp::Time stamp;
  };

  // Replace newest matching track (within match_radius_m_) or append.
  void fuseTrack(const Track & t);

  rclcpp::Subscription<gnn_interfaces::msg::TrackedPolygon>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<Track> tracks_;
  std::mutex mutex_;

  // --- parameters ----------------------------------------------------------
  std::string topic_;
  double a_uncertainty_{1.0};
  double b_fragility_{0.7};
  double k_o11y_cost_{90.0};        // peak cost per cell (<= 200)
  double halo_radius_m_{1.0};
  double match_radius_m_{0.4};      // tracks within this radius are treated as the same object
  double decay_time_s_{20.0};       // drop tracks older than this
  double min_confidence_ignore_{0.0};
  bool   only_raise_{true};

  std::string global_frame_;        // usually "map"

  // Bounds region for updateBounds()
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool   have_bounds_{false};
};

}  // namespace nav2_o11y_risk_layer