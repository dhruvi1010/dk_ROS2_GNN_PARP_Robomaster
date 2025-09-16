#pragma once

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "gnn_interfaces/msg/tracked_polygon.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <filesystem>
#include <fstream>

namespace gnn_objects_layer
{

class GNNObjectsLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  GNNObjectsLayer();
  virtual ~GNNObjectsLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  uint8_t getCostForLabel(uint32_t label, float confidence) const;

  void reset() override;
  bool isClearable() override;
  bool isDiscretized() const;

private:
  void incomingPolygonCallback(const gnn_interfaces::msg::TrackedPolygon::SharedPtr msg);
  void removeStalePolygons();

struct TimedPolygon {
  rclcpp::Time stamp;
  std::vector<geometry_msgs::msg::Point32> points;
  uint32_t label;
  float confidence;
  double custom_decay_time;
    // ✅ Add these two:
  bool first_used = false;
  double first_used_latency_ms = 0.0;
  
};

enum ObjectLabel : uint32_t {
  LABEL_ROBOT = 2,
  LABEL_WORKSTATION = 1,
  LABEL_FORKLIFT = 4,
  LABEL_BOUNDARY = 3
};

  rclcpp::Time last_update_time_;

  std::vector<TimedPolygon> polygons_;
  std::mutex mutex_;

  rclcpp::Subscription<gnn_interfaces::msg::TrackedPolygon>::SharedPtr polygon_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double decay_time_;
  
  std::vector<double> decay_vec;
  std::vector<double> inflation_vec;
  std::string target_frame_;
  std::string topic_;
  std::unordered_map<uint32_t, double> label_decay_times_;
  std::unordered_map<uint32_t, double> label_inflation_radii_;


  double manual_min_x_ = std::numeric_limits<double>::max();
double manual_min_y_ = std::numeric_limits<double>::max();
double manual_max_x_ = std::numeric_limits<double>::lowest();
double manual_max_y_ = std::numeric_limits<double>::lowest();

std::ofstream csv_log_first_use_;
bool first_use_log_initialized_ = false;
std::ofstream csv_log_;
bool csv_logging_enabled_ = true;
bool csv_initialized_ = false;


};

}  // namespace gnn_objects_layer
