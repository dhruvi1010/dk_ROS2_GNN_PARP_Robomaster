#include "gnn_objects_layer/gnn_objects_layer.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include "opencv2/core.hpp"



namespace gnn_objects_layer
{

GNNObjectsLayer::GNNObjectsLayer() {}
static inline std::string utc_iso_now()
{
  using namespace std::chrono;
  auto tp = system_clock::now();
  std::time_t t = system_clock::to_time_t(tp);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
  // append fractional seconds from milliseconds
  auto ms = duration_cast<milliseconds>(tp.time_since_epoch()) % seconds(1);
  oss << '.' << std::setw(3) << std::setfill('0') << ms.count() << 'Z';
  return oss.str();
}

static inline long long wall_epoch_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void GNNObjectsLayer::onInitialize()
{

  rclcpp::sleep_for(std::chrono::milliseconds(1000));  // quick hack

  auto node = node_.lock();
  if (!node) return;

  std::string layer_name = name_;  // e.g., "gnn_costmap_layer"
  //std::string param_ns = layer_name + ".";  // fully scoped
  std::string param_ns = "";
  node->declare_parameter(param_ns + "enabled", true);
  node->get_parameter(param_ns + "enabled", enabled_);

  node->declare_parameter(param_ns + "topic", "/tracked_polygons");
  node->get_parameter(param_ns + "topic", topic_);

// Declare and get the fallback decay time
node->declare_parameter(name_ + ".decay_time", 5.0);  // Default value 5.0
node->get_parameter(name_ + ".decay_time", decay_time_);
RCLCPP_INFO(logger_, "Loaded decay_time: %.2f", decay_time_);
RCLCPP_INFO(logger_, "Layer name: %s", name_.c_str());


node->declare_parameter(name_ + ".label_decay_times", std::vector<double>{});
node->get_parameter(name_ + ".label_decay_times", decay_vec);

for (size_t i = 0; i < decay_vec.size(); ++i) {
  label_decay_times_[static_cast<uint32_t>(i)] = decay_vec[i];
  RCLCPP_INFO(logger_, "Decay for label %u: %.2f", static_cast<uint32_t>(i), decay_vec[i]);
}

node->declare_parameter(name_ + ".label_inflation_radii", std::vector<double>{});
node->get_parameter(name_ + ".label_inflation_radii", inflation_vec);

for (size_t i = 0; i < inflation_vec.size(); ++i) {
  label_inflation_radii_[static_cast<uint32_t>(i)] = inflation_vec[i];
  RCLCPP_INFO(logger_, "Inflation radius for label %u: %.2f", static_cast<uint32_t>(i), inflation_vec[i]);
}

double inflation_radius = 0.0;

// std::vector<double> decay_vec;
// node->declare_parameter("label_decay_times", std::vector<double>{});
// node->get_parameter("label_decay_times", decay_vec);

// for (size_t i = 0; i < decay_vec.size(); ++i) {
//   label_decay_times_[static_cast<uint32_t>(i)] = decay_vec[i];
//   RCLCPP_INFO(logger_, "Decay for label %u: %.2f", i, decay_vec[i]);
// }

// auto params = node->list_parameters({}, 10);
// for (const auto& p : params.names) {
//   RCLCPP_INFO(logger_, "Found param: %s", p.c_str());
// }

  polygon_sub_ = node->create_subscription<gnn_interfaces::msg::TrackedPolygon>(
    topic_, 10,
    std::bind(&GNNObjectsLayer::incomingPolygonCallback, this, std::placeholders::_1)
  );

tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // RCLCPP_INFO(rclcpp::get_logger("GNNObjectsLayer"),
  //   "Initialized GNNObjectsLayer with enabled=%s", enabled_ ? "true" : "false");

  // Mark plugin as valid for publishing
  current_ = true;




  csv_logging_enabled_ = true;  // <--- Toggle this

if (csv_logging_enabled_) {
  std::string base_log_dir = "/workspaces/isaac_ros-dev/gnn_logs/gnn_costmap/";
  std::error_code ec;
  if (!std::filesystem::create_directories(base_log_dir, ec) && ec) {
    RCLCPP_ERROR(logger_, "Failed to create dir %s: %s", base_log_dir.c_str(), ec.message().c_str());
  }

  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss1;
  ss1 << base_log_dir << "gnn_costmap_log_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".csv";
  csv_log_.open(ss1.str(), std::ios::out);
  if (csv_log_.is_open()) {
csv_log_ << "wall_time_iso,wall_epoch_ms,ros_time_ns,ros_time_s,delay_ms,poly_count,update_dt_ms,"
            "label_robot,label_ws,label_fork,label_bound\n";
    csv_initialized_ = true;
    RCLCPP_INFO(logger_, "✅ GNN Costmap logging enabled → %s", ss1.str().c_str());
  } else {
    RCLCPP_ERROR(logger_, "❌ Could not open CSV file: %s", ss1.str().c_str());
  }

  std::stringstream ss2;
  ss2 << base_log_dir << "gnn_costmap_first_use_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".csv";
  csv_log_first_use_.open(ss2.str(), std::ios::out);
  if (csv_log_first_use_.is_open()) {
csv_log_first_use_ << "wall_time_iso,wall_epoch_ms,ros_time_ns,ros_time_s,label,latency_ms\n";
    first_use_log_initialized_ = true;
    RCLCPP_INFO(logger_, "✅ First-use latency logging enabled → %s", ss2.str().c_str());
  } else {
    RCLCPP_ERROR(logger_, "❌ Could not open first-use CSV: %s", ss2.str().c_str());
  }
}




}

void GNNObjectsLayer::incomingPolygonCallback(const gnn_interfaces::msg::TrackedPolygon::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_WARN(rclcpp::get_logger("GNNObjectsLayer"), "Failed to lock node_");
    return;
  }

  rclcpp::Time now = node->now();
  rclcpp::Time msg_time(msg->header.stamp);
  double delay = (now - msg_time).seconds();

  if (delay > 1.0) {
    RCLCPP_WARN(rclcpp::get_logger("GNNObjectsLayer"),
      "Skipping polygon: message is too old (%.2fs)", delay);
    return;
  } else if (delay > 0.3) {
    RCLCPP_WARN(rclcpp::get_logger("GNNObjectsLayer"),
      "Polygon message is delayed by %.2fs — may cause TF errors", delay);
  }


  // ✅ ADD THIS
  // RCLCPP_INFO(rclcpp::get_logger("GNNObjectsLayer"),
  //   "Received polygon with label=%u and confidence=%.2f (%s)",
  //   msg->label, msg->confidence,
  //   (msg->label == LABEL_ROBOT ? "ROBOT" :
  //    msg->label == LABEL_WORKSTATION ? "WORKSTATION" :
  //    msg->label == LABEL_FORKLIFT ? "FORKLIFT" :
  //    msg->label == LABEL_BOUNDARY ? "BOUNDARY" : "UNKNOWN"));



  std::string target_frame = layered_costmap_->getGlobalFrameID();

  geometry_msgs::msg::TransformStamped transform;
  auto tf_time = tf2::TimePoint(
    std::chrono::nanoseconds(
      static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL +
      static_cast<int64_t>(msg->header.stamp.nanosec)));

  try {
    if (!tf_buffer_->canTransform(target_frame, msg->header.frame_id, tf_time,
        tf2::durationFromSec(0.3))) {
      RCLCPP_WARN(rclcpp::get_logger("GNNObjectsLayer"),
        "Transform not available within timeout (0.2s) from %s to %s",
        msg->header.frame_id.c_str(), target_frame.c_str());
      return;
    }

    transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf_time);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(rclcpp::get_logger("GNNObjectsLayer"),
      "TF transform from %s to %s failed at time %.2f: %s",
      msg->header.frame_id.c_str(), target_frame.c_str(),
      msg_time.seconds(), ex.what());
    return;
  }

  // RCLCPP_INFO(rclcpp::get_logger("GNNObjectsLayer"),
  //   "Transform applied: %s → %s | delay: %.2fs",
  //   msg->header.frame_id.c_str(), target_frame.c_str(), delay);

  TimedPolygon tp;
  tp.stamp = msg_time;
  tp.label = msg->label;
  tp.confidence = msg->confidence;
  tp.custom_decay_time = label_decay_times_.count(tp.label) ? label_decay_times_[tp.label] : decay_time_;

  for (const auto &pt : msg->polygon.points) {
    geometry_msgs::msg::Point32 transformed_pt;
    tf2::doTransform(pt, transformed_pt, transform);
    tp.points.push_back(transformed_pt);
  }

  // RCLCPP_INFO(logger_, "Polygon received: label=%u, points=%zu",
  //             tp.label, tp.points.size());

  std::lock_guard<std::mutex> lock(mutex_);
  polygons_.push_back(tp);
// RCLCPP_INFO(logger_, "Polygon received and pushed to buffer: label=%d, points=%zu", msg->label, msg->polygon.points.size());
// RCLCPP_INFO(logger_, "Polygons buffer size: %zu", polygons_.size());

}



void GNNObjectsLayer::removeStalePolygons()
{
  

//size_t before = polygons_.size();
  std::lock_guard<std::mutex> lock(mutex_);
rclcpp::Time now = node_.lock()->now();
//RCLCPP_INFO(logger_, "Checking %zu polygons at time %.2f", polygons_.size(), now.seconds());
polygons_.erase(std::remove_if(polygons_.begin(), polygons_.end(),
  [now](const TimedPolygon &p) {
    return (now - p.stamp).seconds() > p.custom_decay_time;
  }), polygons_.end());
  // RCLCPP_INFO(rclcpp::get_logger("GNNObjectsLayer"),
  // "Checking %zu polygons for expiration at time %.2f", polygons_.size(), now.seconds());
//RCLCPP_INFO(logger_, "Remaining after filtering: %zu", polygons_.size());
}
void GNNObjectsLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) {
    // Layer is idle but healthy.
    current_ = true;
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);



  for (const auto& poly : polygons_) {
    if (poly.points.empty()) continue;

    for (const auto& pt : poly.points) {
      *min_x = std::min(*min_x, static_cast<double>(pt.x));
      *min_y = std::min(*min_y, static_cast<double>(pt.y));
      *max_x = std::max(*max_x, static_cast<double>(pt.x));
      *max_y = std::max(*max_y, static_cast<double>(pt.y));
    }

    //bounds_updated = true;
  }

  // Optional: Handle fallback dirty area if no polygons are present
  // if (!bounds_updated) {
  //   *min_x = std::min(*min_x, manual_min_x_);
  //   *min_y = std::min(*min_y, manual_min_y_);
  //   *max_x = std::max(*max_x, manual_max_x_);
  //   *max_y = std::max(*max_y, manual_max_y_);
  // }

  // Optional debug log:
  // if (bounds_updated) {
  //   RCLCPP_DEBUG(logger_,
  //     "[GNN Costmap] updateBounds → min: (%.2f, %.2f), max: (%.2f, %.2f)",
  //     *min_x, *min_y, *max_x, *max_y);
  // }

  current_ = true;
}


uint8_t GNNObjectsLayer::getCostForLabel(uint32_t label, float confidence) const
{
  switch (label) {
    case LABEL_ROBOT:
    case LABEL_WORKSTATION:
    case LABEL_BOUNDARY:
      return nav2_costmap_2d::LETHAL_OBSTACLE;  // 254

    case LABEL_FORKLIFT:
      return (confidence > 0.8f) ? 200 : 100;

    default:
      return 0;  // Unused — already filtered
  }
}

void GNNObjectsLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int, int, int, int)
{
  if (!enabled_) return;
  auto node = node_.lock();
  if (!node) return;

  removeStalePolygons();
  std::lock_guard<std::mutex> lock(mutex_);

  static cv::Mat last_cost_mask;

  const double origin_x = master_grid.getOriginX();
  const double origin_y = master_grid.getOriginY();
  const int width = master_grid.getSizeInCellsX();
  const int height = master_grid.getSizeInCellsY();
  const double resolution = master_grid.getResolution();
  const double max_x = origin_x + width * resolution;
  const double max_y = origin_y + height * resolution;

  // Clear old costs
  if (!last_cost_mask.empty()) {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (last_cost_mask.at<uchar>(y, x) > 0) {
          master_grid.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
        }
      }
    }
  }

  rclcpp::Time now = node->now();
  double update_dt = last_update_time_.nanoseconds() > 0 ?
    (now - last_update_time_).seconds() * 1000.0 : 0.0;
  last_update_time_ = now;

  double max_age_ms = 0.0;
  std::map<uint32_t, int> label_counts;

  // Build new masks
  cv::Mat base_mask(height, width, CV_8UC1, cv::Scalar(0));
  cv::Mat cost_mask(height, width, CV_8UC1, cv::Scalar(0));

  for (auto& poly : polygons_) {
    double age_ms = (now - poly.stamp).seconds() * 1000.0;
    max_age_ms = std::max(max_age_ms, age_ms);
    label_counts[poly.label]++;


    if (poly.points.size() < 3) continue;

    std::vector<cv::Point> contour;
    for (const auto& pt : poly.points) {
      if (pt.x < origin_x || pt.x >= max_x || pt.y < origin_y || pt.y >= max_y) continue;
      unsigned int mx, my;
      if (master_grid.worldToMap(pt.x, pt.y, mx, my)) {
        contour.emplace_back(mx, my);
      }
    }

    if (contour.size() < 3) continue;

    //Logging
    if (first_use_log_initialized_) {
    rclcpp::Time ros_now = node->now();
    const int64_t ros_ns = ros_now.nanoseconds();
    const double  ros_s  = ros_now.seconds();

    csv_log_first_use_ << utc_iso_now() << ","
                       << wall_epoch_ms() << ","
                       << ros_ns << ","
                       << std::fixed << std::setprecision(9) << ros_s << ","
                       << poly.label << ","
                       << std::fixed << std::setprecision(3) << age_ms << "\n";
    csv_log_first_use_.flush();
  }

    std::vector<std::vector<cv::Point>> contours = {contour};

    cv::fillPoly(base_mask, contours, cv::Scalar(255));
    cv::fillPoly(cost_mask, contours, cv::Scalar(nav2_costmap_2d::LETHAL_OBSTACLE));

    double inflation_radius = 0.0;
    if (label_inflation_radii_.count(poly.label)) {
      inflation_radius = label_inflation_radii_.at(poly.label);
    }

    int inflation_px = static_cast<int>(inflation_radius / resolution);
    if (inflation_px <= 0) continue;

    cv::Mat dt_mask;
    cv::distanceTransform(255 - base_mask, dt_mask, cv::DIST_L2, 3);

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (base_mask.at<uchar>(y, x) == 255) continue;

        float dist = dt_mask.at<float>(y, x);
        if (dist <= inflation_px) {
          float ratio = std::max(0.0f, 1.0f - dist / inflation_px);
          uint8_t infl_cost = static_cast<uint8_t>(ratio * 253);
          cost_mask.at<uchar>(y, x) = std::max(cost_mask.at<uchar>(y, x), infl_cost);
        }
      }
    }
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      uint8_t new_cost = cost_mask.at<uchar>(y, x);
      if (new_cost > 0 && master_grid.getCost(x, y) < new_cost) {
        master_grid.setCost(x, y, new_cost);
      }
    }
  }

  last_cost_mask = cost_mask.clone();  // Save for clearing in next update

  // Logging
if (csv_logging_enabled_ && csv_initialized_) {
  // clocks
  rclcpp::Time ros_now = node->now();               // ROS time (sim or system, depending on /clock)
  const int64_t ros_ns = ros_now.nanoseconds();     // exact nanoseconds
  const double  ros_s  = ros_now.seconds();         // double seconds (for quick visuals)

  csv_log_ << utc_iso_now() << ","
           << wall_epoch_ms() << ","
           << ros_ns << ","
           << std::fixed << std::setprecision(9) << ros_s << ","
           << std::fixed << std::setprecision(3) << max_age_ms << ","
           << polygons_.size() << ","
           << update_dt << ","
           << label_counts[1] << "," << label_counts[2] << ","
           << label_counts[3] << "," << label_counts[4] << "\n";
  csv_log_.flush();
}

  current_ = true;
}


bool GNNObjectsLayer::isClearable()
{
  return true;
}

bool GNNObjectsLayer::isDiscretized() const {
  return true;
}

void GNNObjectsLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  polygons_.clear();
}

}  // namespace gnn_objects_layer
// PLUGINLIB_EXPORT_CLASS(gnn_objects_layer::GNNObjectsLayer, nav2_costmap_2d::CostmapLayer)
PLUGINLIB_EXPORT_CLASS(gnn_objects_layer::GNNObjectsLayer, nav2_costmap_2d::Layer)
