#include "nav2_comms_risk_layer/comms_risk_layer.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

PLUGINLIB_EXPORT_CLASS(
  nav2_comms_risk_layer::CommsRiskLayer,
  nav2_costmap_2d::Layer)

namespace nav2_comms_risk_layer
{

CommsRiskLayer::CommsRiskLayer() = default;

void CommsRiskLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("CommsRiskLayer"), "Node handle is null");
    return;
  }

  // --- parameters ----------------------------------------------------------
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("comms/link_stats")));
  declareParameter("alpha", rclcpp::ParameterValue(0.2));
  declareParameter("rsrp_min_dbm", rclcpp::ParameterValue(-85.0));
  declareParameter("rsrp_range_db", rclcpp::ParameterValue(25.0));
  declareParameter("jitter_max_ms", rclcpp::ParameterValue(30.0));
  declareParameter("jitter_range_ms", rclcpp::ParameterValue(40.0));
  declareParameter("w_rsrp", rclcpp::ParameterValue(0.6));
  declareParameter("w_jitter", rclcpp::ParameterValue(0.4));
  declareParameter("w_loss", rclcpp::ParameterValue(0.0));
  declareParameter("k_cost", rclcpp::ParameterValue(120.0));
  declareParameter("inflation_radius_m", rclcpp::ParameterValue(1.2));
  declareParameter("decay_time_s", rclcpp::ParameterValue(30.0));
  declareParameter("only_raise", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".alpha", alpha_);
  node->get_parameter(name_ + ".rsrp_min_dbm", rsrp_min_dbm_);
  node->get_parameter(name_ + ".rsrp_range_db", rsrp_range_db_);
  node->get_parameter(name_ + ".jitter_max_ms", jitter_max_ms_);
  node->get_parameter(name_ + ".jitter_range_ms", jitter_range_ms_);
  node->get_parameter(name_ + ".w_rsrp", w_rsrp_);
  node->get_parameter(name_ + ".w_jitter", w_jitter_);
  node->get_parameter(name_ + ".w_loss", w_loss_);
  node->get_parameter(name_ + ".k_cost", k_cost_);
  node->get_parameter(name_ + ".inflation_radius_m", inflation_radius_m_);
  node->get_parameter(name_ + ".decay_time_s", decay_time_s_);
  node->get_parameter(name_ + ".only_raise", only_raise_);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  sub_ = node->create_subscription<perception_aware_nav2_msgs::msg::LinkStats>(
    topic_, rclcpp::QoS(10),
    std::bind(&CommsRiskLayer::linkStatsCallback, this, std::placeholders::_1));

  current_ = true;
/*   RCLCPP_INFO(node->get_logger(),
    "[CommsRiskLayer] subscribed to '%s' in frame '%s' "
    "(alpha=%.2f k_cost=%.0f inflation=%.2fm decay=%.0fs)",
    topic_.c_str(), global_frame_.c_str(),
    alpha_, k_cost_, inflation_radius_m_, decay_time_s_); */

  RCLCPP_INFO(node->get_logger(),
    "\033[36m[CommsRiskLayer] subscribed to '%s' in frame '%s' "
    "(alpha=%.2f k_cost=%.0f inflation=%.2fm decay=%.0fs)\033[0m",
    topic_.c_str(), global_frame_.c_str(),
    alpha_, k_cost_, inflation_radius_m_, decay_time_s_);
   
}

double CommsRiskLayer::riskFromStats(
  const perception_aware_nav2_msgs::msg::LinkStats & s) const
{
  auto clamp01 = [](double v){ return std::clamp(v, 0.0, 1.0); };

  double r_rsrp = 0.0, r_jit = 0.0, r_loss = 0.0;

  if (std::isfinite(s.rsrp_dbm)) {
    // Worse (more negative) RSRP -> higher risk.
    r_rsrp = clamp01((rsrp_min_dbm_ - s.rsrp_dbm) / rsrp_range_db_);
  }
  if (std::isfinite(s.jitter_ms)) {
    r_jit = clamp01((s.jitter_ms - jitter_max_ms_) / jitter_range_ms_);
  }
  if (std::isfinite(s.loss_rate)) {
    r_loss = clamp01(static_cast<double>(s.loss_rate));
  }

  double r = w_rsrp_ * r_rsrp + w_jitter_ * r_jit + w_loss_ * r_loss;
  return clamp01(r);
}

void CommsRiskLayer::linkStatsCallback(
  const perception_aware_nav2_msgs::msg::LinkStats::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  // // Look up the robot pose in the global frame at the time we got the stats.
  // geometry_msgs::msg::PoseStamped robot_pose;
  // if (!layered_costmap_->getRobotPose(robot_pose)) {
  //   RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
  //     "[CommsRiskLayer] failed to get robot pose; dropping sample");
  //   return;
  // }

  // double risk = riskFromStats(*msg);
  // rclcpp::Time t = node->now();

  // fuseSample(robot_pose.pose.position.x,
  //            robot_pose.pose.position.y,
  //            risk, t);
  // ---------------------------------
  // Reuse the robot pose that updateBounds() most recently cached.
  // (nav2_costmap_2d::Layer doesn't expose getRobotPose; updateBounds is
  //  the canonical hand-off for current pose into a layer.)
  double rx, ry;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_robot_pose_) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
        "[CommsRiskLayer] no robot pose cached yet "
        "(waiting for first updateBounds); dropping sample");
      return;
    }
    rx = last_robot_x_;
    ry = last_robot_y_;
  }

  double risk = riskFromStats(*msg);
  rclcpp::Time t = node->now();

  fuseSample(rx, ry, risk, t);

  // log the raw stats and risk for debugging / tuning
/*     RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
    "[CommsRiskLayer] sample @ (%.2f, %.2f) risk=%.3f rsrp=%.1f dBm jit=%.1f ms",
    rx, ry, risk, msg->rsrp_dbm, msg->jitter_ms); */
    
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
    "\033[36m[CommsRiskLayer] sample @ (%.2f, %.2f) risk=%.3f rsrp=%.1f dBm jit=%.1f ms\033[0m",
    rx, ry, risk, msg->rsrp_dbm, msg->jitter_ms);

}

void CommsRiskLayer::fuseSample(
  double wx, double wy, double risk, const rclcpp::Time & t)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Find nearest existing sample within inflation_radius_m_; EMA-update it.
  const double r2 = inflation_radius_m_ * inflation_radius_m_;
  Sample * nearest = nullptr;
  double best = r2;

  for (auto & s : samples_) {
    double dx = s.wx - wx, dy = s.wy - wy;
    double d2 = dx * dx + dy * dy;
    if (d2 < best) {
      best = d2;
      nearest = &s;
    }
  }

  if (nearest) {
    nearest->risk = (1.0 - alpha_) * nearest->risk + alpha_ * risk;
    nearest->stamp = t;
  } else {
    samples_.push_back({wx, wy, risk, t});
  }

  // Track dirty region for updateBounds()
  double r = inflation_radius_m_;
  if (!have_bounds_) {
    last_min_x_ = wx - r;  last_min_y_ = wy - r;
    last_max_x_ = wx + r;  last_max_y_ = wy + r;
    have_bounds_ = true;
  } else {
    last_min_x_ = std::min(last_min_x_, wx - r);
    last_min_y_ = std::min(last_min_y_, wy - r);
    last_max_x_ = std::max(last_max_x_, wx + r);
    last_max_y_ = std::max(last_max_y_, wy + r);
  }
}

void CommsRiskLayer::updateBounds(
//   double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
//   double * min_x, double * min_y, double * max_x, double * max_y)
// {
//   if (!enabled_) return;

//   std::lock_guard<std::mutex> lock(mutex_);

  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mutex_);

    // Cache the current pose for the async LinkStats callback to use.
  last_robot_x_ = robot_x;
  last_robot_y_ = robot_y;
  have_robot_pose_ = true;

  // Age out stale samples here so cells can decay even without new messages.
  auto node = node_.lock();
  if (node && decay_time_s_ > 0.0) {
    rclcpp::Time now = node->now();
    samples_.erase(std::remove_if(samples_.begin(), samples_.end(),
      [&](const Sample & s){
        return (now - s.stamp).seconds() > decay_time_s_;
      }), samples_.end());
  }

  //Log the number of active samples for debugging / tuning.
/*   if (node) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
      "[CommsRiskLayer] active samples: %zu", samples_.size());
  } */

  if (node) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
      "\033[36m[CommsRiskLayer] active samples: %zu\033[0m", samples_.size());
  }


  if (!have_bounds_ || samples_.empty()) return;

  *min_x = std::min(*min_x, last_min_x_);
  *min_y = std::min(*min_y, last_min_y_);
  *max_x = std::max(*max_x, last_max_x_);
  *max_y = std::max(*max_y, last_max_y_);
}

void CommsRiskLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty()) return;

  const double res = master_grid.getResolution();
  const double r   = inflation_radius_m_;
  const double r2  = r * r;
  const unsigned char cap =
    static_cast<unsigned char>(std::min(252.0, k_cost_));

  for (const auto & s : samples_) {
    if (s.risk <= 0.0) continue;

    // Convert world center to map bounds in cells
    unsigned int cx, cy;
    if (!master_grid.worldToMap(s.wx, s.wy, cx, cy)) continue;

    int dcells = static_cast<int>(std::ceil(r / res));
    int xi_min = std::max<int>(min_i, static_cast<int>(cx) - dcells);
    int xi_max = std::min<int>(max_i, static_cast<int>(cx) + dcells);
    int yj_min = std::max<int>(min_j, static_cast<int>(cy) - dcells);
    int yj_max = std::min<int>(max_j, static_cast<int>(cy) + dcells);

    for (int yj = yj_min; yj < yj_max; ++yj) {
      for (int xi = xi_min; xi < xi_max; ++xi) {
        double wx, wy;
        master_grid.mapToWorld(xi, yj, wx, wy);
        double dx = wx - s.wx, dy = wy - s.wy;
        double d2 = dx * dx + dy * dy;
        if (d2 > r2) continue;

        double falloff = 1.0 - std::sqrt(d2) / r;            // 1.0 at center → 0 at edge
        double cell_cost_d = k_cost_ * s.risk * falloff;
        unsigned char cell_cost =
          static_cast<unsigned char>(std::clamp(cell_cost_d, 0.0,
                                                static_cast<double>(cap)));
        if (cell_cost == 0) continue;

        unsigned char existing = master_grid.getCost(xi, yj);

        // Never touch true obstacles / inscribed inflation from other layers.
        if (existing == nav2_costmap_2d::LETHAL_OBSTACLE ||
            existing == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) continue;

        if (only_raise_) {
          if (cell_cost > existing) {
            master_grid.setCost(xi, yj, cell_cost);
          }
        } else {
          unsigned sum = static_cast<unsigned>(existing) + cell_cost;
          master_grid.setCost(xi, yj,
            static_cast<unsigned char>(std::min<unsigned>(sum, 252u)));
        }
      }
    }
  }
}

void CommsRiskLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
  have_bounds_ = false;
  have_robot_pose_ = false;
}

}  // namespace nav2_comms_risk_layer