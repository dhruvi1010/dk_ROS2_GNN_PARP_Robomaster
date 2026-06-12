#include "nav2_safety_risk_layer/safety_risk_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nav2_safety_risk_layer::SafetyRiskLayer,
  nav2_costmap_2d::Layer)

namespace nav2_safety_risk_layer
{

SafetyRiskLayer::SafetyRiskLayer() = default;

void SafetyRiskLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("SafetyRiskLayer"), "\033[31mNode handle is null\033[0m");
    return;
  }

  // ---- parameter declarations -------------------------------------------
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("tracked_polygons_topic",
                   rclcpp::ParameterValue(std::string("/tracked_polygons")));
  declareParameter("dynamic_labels",
                   rclcpp::ParameterValue(std::vector<int64_t>{2, 4}));
  declareParameter("match_radius_m",       rclcpp::ParameterValue(0.4));
  declareParameter("decay_time_s",         rclcpp::ParameterValue(20.0));
  declareParameter("history_window_s",     rclcpp::ParameterValue(2.0));
  declareParameter("max_history",          rclcpp::ParameterValue(8));
  declareParameter("safety_horizon_s",     rclcpp::ParameterValue(3.0));
  declareParameter("safety_dt_s",          rclcpp::ParameterValue(0.3));
  declareParameter("halo_radius_m",        rclcpp::ParameterValue(0.6));
  declareParameter("k_safety_cost",        rclcpp::ParameterValue(100.0));
  declareParameter("fade_with_horizon",    rclcpp::ParameterValue(true));
  declareParameter("only_raise",           rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".enabled",                enabled_);
  node->get_parameter(name_ + ".tracked_polygons_topic", topic_);
  node->get_parameter(name_ + ".dynamic_labels",         dynamic_labels_);
  node->get_parameter(name_ + ".match_radius_m",         match_radius_m_);
  node->get_parameter(name_ + ".decay_time_s",           decay_time_s_);
  node->get_parameter(name_ + ".history_window_s",       history_window_s_);
  int mh = 8;
  node->get_parameter(name_ + ".max_history",            mh);
  max_history_ = static_cast<size_t>(std::max(2, mh));
  node->get_parameter(name_ + ".safety_horizon_s",       safety_horizon_s_);
  node->get_parameter(name_ + ".safety_dt_s",            safety_dt_s_);
  node->get_parameter(name_ + ".halo_radius_m",          halo_radius_m_);
  node->get_parameter(name_ + ".k_safety_cost",          k_safety_cost_);
  node->get_parameter(name_ + ".fade_with_horizon",      fade_with_horizon_);
  node->get_parameter(name_ + ".only_raise",             only_raise_);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  sub_ = node->create_subscription<gnn_interfaces::msg::TrackedPolygon>(
    topic_, rclcpp::QoS(10),
    std::bind(&SafetyRiskLayer::polygonCallback, this, std::placeholders::_1));

  current_ = true;

  std::stringstream ss;
  for (auto l : dynamic_labels_) ss << l << ",";
  RCLCPP_INFO(node->get_logger(),
  "\033[33m[SafetyRiskLayer] up. topic='%s' frame='%s' dyn_labels=[%s] "
  "H=%.1fs dt=%.2fs halo=%.2fm k=%.0f fade=%d match=%.2f\033[0m",
  topic_.c_str(), global_frame_.c_str(), ss.str().c_str(),
  safety_horizon_s_, safety_dt_s_, halo_radius_m_, k_safety_cost_,
  static_cast<int>(fade_with_horizon_), match_radius_m_);

}

bool SafetyRiskLayer::isDynamicLabel(uint32_t label) const
{
  for (auto l : dynamic_labels_) {
    if (static_cast<int64_t>(label) == l) return true;
  }
  return false;
}

bool SafetyRiskLayer::estimateVelocity(
  const Track & tr, double & vx, double & vy) const
{
  if (tr.history.size() < 2) { vx = vy = 0.0; return false; }
  const auto & p0 = tr.history.front();
  const auto & pN = tr.history.back();
  double dt = (pN.stamp - p0.stamp).seconds();
  if (dt < 0.1) { vx = vy = 0.0; return false; }
  vx = (pN.cx - p0.cx) / dt;
  vy = (pN.cy - p0.cy) / dt;
  return true;
}

void SafetyRiskLayer::prune(const rclcpp::Time & now)
{
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
    [&](const Track & tr) {
      return (now - tr.last_t).seconds() > decay_time_s_;
    }), tracks_.end());
}

void SafetyRiskLayer::polygonCallback(
  const gnn_interfaces::msg::TrackedPolygon::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  // Frame guard (Lf3_Lf4 §2.5: confirmed 'map' in practice).
  if (!msg->header.frame_id.empty() &&
      msg->header.frame_id != global_frame_) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
  "\033[33m[SafetyRiskLayer] dropping polygon in frame '%s' (expected '%s')\033[0m",
  msg->header.frame_id.c_str(), global_frame_.c_str());

    return;
  }

  // Label filter — static labels never feed L3.
  if (!isDynamicLabel(msg->label)) return;

  if (msg->polygon.points.empty()) return;

  // Centroid (drop closed-ring duplicate if first == last).
  size_t n = msg->polygon.points.size();
  if (n >= 2 &&
      std::abs(msg->polygon.points.front().x -
               msg->polygon.points.back().x) < 1e-9 &&
      std::abs(msg->polygon.points.front().y -
               msg->polygon.points.back().y) < 1e-9) {
    --n;
  }
  if (n == 0) return;

  double sx = 0.0, sy = 0.0;
  for (size_t i = 0; i < n; ++i) {
    sx += msg->polygon.points[i].x;
    sy += msg->polygon.points[i].y;
  }
  double cx = sx / static_cast<double>(n);
  double cy = sy / static_cast<double>(n);

  rclcpp::Time stamp = node->now();

  std::lock_guard<std::mutex> lock(mutex_);

  // Match-and-replace by nearest centroid within match_radius_m.
  Track * nearest = nullptr;
  double best_d2 = match_radius_m_ * match_radius_m_;
  for (auto & tr : tracks_) {
    double dx = tr.cx - cx;
    double dy = tr.cy - cy;
    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) { best_d2 = d2; nearest = &tr; }
  }

  if (nearest) {
    nearest->cx = cx;
    nearest->cy = cy;
    nearest->label = msg->label;
    nearest->last_t = stamp;
    nearest->history.push_back({stamp, cx, cy});
    while (!nearest->history.empty() &&
           (stamp - nearest->history.front().stamp).seconds() > history_window_s_) {
      nearest->history.pop_front();
    }
    while (nearest->history.size() > max_history_) {
      nearest->history.pop_front();
    }
  } else {
    Track t;
    t.label = msg->label;
    t.cx = cx;
    t.cy = cy;
    t.last_t = stamp;
    t.history.push_back({stamp, cx, cy});
    tracks_.push_back(std::move(t));
  }
}

void SafetyRiskLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;

  auto node = node_.lock();
  if (!node) return;
  rclcpp::Time now = node->now();

  std::lock_guard<std::mutex> lock(mutex_);
  prune(now);

  if (tracks_.empty()) {
    have_bounds_ = false;
    return;
  }

  // Dirty region = union over predicted trajectory points ± halo_radius.
  double lo_x =  std::numeric_limits<double>::infinity();
  double lo_y =  std::numeric_limits<double>::infinity();
  double hi_x = -std::numeric_limits<double>::infinity();
  double hi_y = -std::numeric_limits<double>::infinity();

  for (const auto & tr : tracks_) {
    double vx = 0.0, vy = 0.0;
    estimateVelocity(tr, vx, vy);
    for (double t = 0.0; t <= safety_horizon_s_; t += safety_dt_s_) {
      double px = tr.cx + vx * t;
      double py = tr.cy + vy * t;
      lo_x = std::min(lo_x, px - halo_radius_m_);
      lo_y = std::min(lo_y, py - halo_radius_m_);
      hi_x = std::max(hi_x, px + halo_radius_m_);
      hi_y = std::max(hi_y, py + halo_radius_m_);
    }
  }

  last_min_x_ = lo_x; last_min_y_ = lo_y;
  last_max_x_ = hi_x; last_max_y_ = hi_y;
  have_bounds_ = true;

  *min_x = std::min(*min_x, lo_x);
  *min_y = std::min(*min_y, lo_y);
  *max_x = std::max(*max_x, hi_x);
  *max_y = std::max(*max_y, hi_y);
}

void SafetyRiskLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mutex_);
  if (tracks_.empty()) return;

  const double res  = master_grid.getResolution();
  const double r    = halo_radius_m_;
  const double r2   = r * r;
  const unsigned char cap =
    static_cast<unsigned char>(std::min(252.0, k_safety_cost_));
  const int dcells  = std::max(1, static_cast<int>(std::ceil(r / res)));

  size_t painted_tracks = 0;

  for (const auto & tr : tracks_) {
    double vx = 0.0, vy = 0.0;
    estimateVelocity(tr, vx, vy);

    for (double t = 0.0; t <= safety_horizon_s_; t += safety_dt_s_) {
      double px = tr.cx + vx * t;
      double py = tr.cy + vy * t;

      unsigned int ux, uy;
      if (!master_grid.worldToMap(px, py, ux, uy)) continue;

      double peak = k_safety_cost_;
      if (fade_with_horizon_ && safety_horizon_s_ > 1e-3) {
        peak *= std::max(0.0, 1.0 - t / safety_horizon_s_);
      }
      if (peak <= 0.0) continue;

      int xi_min = std::max<int>(min_i, static_cast<int>(ux) - dcells);
      int xi_max = std::min<int>(max_i, static_cast<int>(ux) + dcells);
      int yj_min = std::max<int>(min_j, static_cast<int>(uy) - dcells);
      int yj_max = std::min<int>(max_j, static_cast<int>(uy) + dcells);

      for (int yj = yj_min; yj < yj_max; ++yj) {
        for (int xi = xi_min; xi < xi_max; ++xi) {
          double wx, wy;
          master_grid.mapToWorld(xi, yj, wx, wy);
          double dx = wx - px, dy = wy - py;
          double d2 = dx * dx + dy * dy;
          if (d2 > r2) continue;

          double falloff = 1.0 - std::sqrt(d2) / r;
          double cell_cost_d = peak * falloff;
          unsigned char cell_cost =
            static_cast<unsigned char>(std::clamp(cell_cost_d, 0.0,
                                                  static_cast<double>(cap)));
          if (cell_cost == 0) continue;

          unsigned char existing = master_grid.getCost(xi, yj);
          if (existing == nav2_costmap_2d::LETHAL_OBSTACLE ||
              existing == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) continue;

          if (only_raise_) {
            if (cell_cost > existing) master_grid.setCost(xi, yj, cell_cost);
          } else {
            unsigned sum = static_cast<unsigned>(existing) + cell_cost;
            master_grid.setCost(xi, yj,
              static_cast<unsigned char>(std::min<unsigned>(sum, 252u)));
          }
        }
      }
    }
    ++painted_tracks;
  }

  auto node = node_.lock();
  if (node) {
    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
  "\033[33m[SafetyRiskLayer] active tracks: %zu  painted: %zu\033[0m",
  tracks_.size(), painted_tracks);

  }
}

void SafetyRiskLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  tracks_.clear();
  have_bounds_ = false;
}

}  // namespace nav2_safety_risk_layer