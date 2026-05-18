#include "nav2_o11y_risk_layer/o11y_risk_layer.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(
  nav2_o11y_risk_layer::O11yRiskLayer,
  nav2_costmap_2d::Layer)

namespace nav2_o11y_risk_layer
{

O11yRiskLayer::O11yRiskLayer() = default;

void O11yRiskLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("O11yRiskLayer"), "\033[31mNode handle is null\033[0m");
    return;
  }

  // --- parameters ----------------------------------------------------------
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("tracked_polygons_topic",
                   rclcpp::ParameterValue(std::string("/tracked_polygons")));
  declareParameter("a_uncertainty",         rclcpp::ParameterValue(1.0));
  declareParameter("b_fragility",           rclcpp::ParameterValue(0.7));
  declareParameter("k_o11y_cost",           rclcpp::ParameterValue(90.0));
  declareParameter("halo_radius_m",         rclcpp::ParameterValue(1.0));
  declareParameter("match_radius_m",        rclcpp::ParameterValue(0.4));
  declareParameter("decay_time_s",          rclcpp::ParameterValue(20.0));
  declareParameter("min_confidence_ignore", rclcpp::ParameterValue(0.0));
  declareParameter("only_raise",            rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".tracked_polygons_topic", topic_);
  node->get_parameter(name_ + ".a_uncertainty", a_uncertainty_);
  node->get_parameter(name_ + ".b_fragility",   b_fragility_);
  node->get_parameter(name_ + ".k_o11y_cost",   k_o11y_cost_);
  node->get_parameter(name_ + ".halo_radius_m", halo_radius_m_);
  node->get_parameter(name_ + ".match_radius_m", match_radius_m_);
  node->get_parameter(name_ + ".decay_time_s",  decay_time_s_);
  node->get_parameter(name_ + ".min_confidence_ignore", min_confidence_ignore_);
  node->get_parameter(name_ + ".only_raise",    only_raise_);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_ = node->create_subscription<gnn_interfaces::msg::TrackedPolygon>(
    topic_, rclcpp::QoS(10),
    std::bind(&O11yRiskLayer::polygonCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(node->get_logger(),
    "\033[35m[O11yRiskLayer] subscribed to '%s' in frame '%s' "
    "(a_unc=%.2f b_frag=%.2f k=%.0f halo=%.2fm decay=%.0fs)\033[0m",
    topic_.c_str(), global_frame_.c_str(),
    a_uncertainty_, b_fragility_, k_o11y_cost_, halo_radius_m_, decay_time_s_);
  }

double O11yRiskLayer::riskFromPolygon(
  const gnn_interfaces::msg::TrackedPolygon & p) const
{
  auto clamp01 = [](double v){ return std::clamp(v, 0.0, 1.0); };

  // Drop low-confidence detections entirely (optional).
  if (p.confidence < min_confidence_ignore_) {
    return 0.0;
  }

  double uncertainty = clamp01(1.0 - static_cast<double>(p.confidence));

  // fragility = max(contributor_ratios). If only one robot saw it -> 1.0.
  // If empty (defensive), treat as fully fragile.
  double fragility = 1.0;
  if (!p.contributor_ratios.empty()) {
    fragility = 0.0;
    for (float r : p.contributor_ratios) {
      fragility = std::max(fragility, static_cast<double>(r));
    }
    fragility = clamp01(fragility);
  }

  double risk = a_uncertainty_ * uncertainty + b_fragility_ * fragility;
  // Normalize so result stays in [0, 1] for the cost-scale step.
  double denom = std::max(1e-6, a_uncertainty_ + b_fragility_);
  return clamp01(risk / denom);
}

void O11yRiskLayer::polygonCallback(
  const gnn_interfaces::msg::TrackedPolygon::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  if (msg->polygon.points.empty()) return;

  // ---- transform polygon points to the global frame (`map`) ---------------
  geometry_msgs::msg::TransformStamped tf;
  try {
    if (!tf_buffer_->canTransform(
          global_frame_, msg->header.frame_id,
          tf2::TimePointZero, tf2::durationFromSec(0.2)))
    {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
        "\033[35m[O11yRiskLayer] cannot transform %s -> %s (yet)\033[0m",
        msg->header.frame_id.c_str(), global_frame_.c_str());
      return;
    }
    tf = tf_buffer_->lookupTransform(
        global_frame_, msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
      "\033[35m[O11yRiskLayer] TF lookup failed: %s\033[0m", ex.what());
    return;
  }

  // ---- compute centroid in `map` frame ------------------------------------
  double sx = 0.0, sy = 0.0;
  size_t n = 0;
  for (const auto & pt : msg->polygon.points) {
    geometry_msgs::msg::Point32 out;
    tf2::doTransform(pt, out, tf);
    sx += out.x; sy += out.y; ++n;
  }
  if (n == 0) return;
  double cx = sx / static_cast<double>(n);
  double cy = sy / static_cast<double>(n);

  // ---- risk + fuse --------------------------------------------------------
  double risk = riskFromPolygon(*msg);

  Track t;
  t.cx = cx; t.cy = cy;
  t.risk  = risk;
  t.label = msg->label;
  t.stamp = node->now();

  fuseTrack(t);
}

void O11yRiskLayer::fuseTrack(const Track & t)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Replace the closest existing track within match_radius_m_, else append.
  const double r2 = match_radius_m_ * match_radius_m_;
  Track * nearest = nullptr;
  double best = r2;
  for (auto & s : tracks_) {
    double dx = s.cx - t.cx, dy = s.cy - t.cy;
    double d2 = dx * dx + dy * dy;
    if (d2 < best) { best = d2; nearest = &s; }
  }
  if (nearest) {
    *nearest = t;          // overwrite — TrackedPolygon already carries fresh state
  } else {
    tracks_.push_back(t);
  }

  // Track dirty region for updateBounds()
  double r = halo_radius_m_;
  double mnx = t.cx - r, mny = t.cy - r;
  double mxx = t.cx + r, mxy = t.cy + r;
  if (!have_bounds_) {
    last_min_x_ = mnx; last_min_y_ = mny;
    last_max_x_ = mxx; last_max_y_ = mxy;
    have_bounds_ = true;
  } else {
    last_min_x_ = std::min(last_min_x_, mnx);
    last_min_y_ = std::min(last_min_y_, mny);
    last_max_x_ = std::max(last_max_x_, mxx);
    last_max_y_ = std::max(last_max_y_, mxy);
  }
}

void O11yRiskLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mutex_);

  // Age out stale tracks so cost fades when GNN stops emitting.
  auto node = node_.lock();
  if (node && decay_time_s_ > 0.0) {
    rclcpp::Time now = node->now();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
      [&](const Track & s){
        return (now - s.stamp).seconds() > decay_time_s_;
      }), tracks_.end());
  }

  if (!have_bounds_ || tracks_.empty()) return;

  *min_x = std::min(*min_x, last_min_x_);
  *min_y = std::min(*min_y, last_min_y_);
  *max_x = std::max(*max_x, last_max_x_);
  *max_y = std::max(*max_y, last_max_y_);
}

void O11yRiskLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(mutex_);
  if (tracks_.empty()) return;

  const double res = master_grid.getResolution();
  const double r   = halo_radius_m_;
  const double r2  = r * r;
  const unsigned char cap =
    static_cast<unsigned char>(std::min(252.0, k_o11y_cost_));

  size_t painted_tracks = 0;

  for (const auto & s : tracks_) {
    if (s.risk <= 0.0) continue;

    unsigned int cx, cy;
    if (!master_grid.worldToMap(s.cx, s.cy, cx, cy)) continue;

    int dcells = static_cast<int>(std::ceil(r / res));
    int xi_min = std::max<int>(min_i, static_cast<int>(cx) - dcells);
    int xi_max = std::min<int>(max_i, static_cast<int>(cx) + dcells);
    int yj_min = std::max<int>(min_j, static_cast<int>(cy) - dcells);
    int yj_max = std::min<int>(max_j, static_cast<int>(cy) + dcells);

    for (int yj = yj_min; yj < yj_max; ++yj) {
      for (int xi = xi_min; xi < xi_max; ++xi) {
        double wx, wy;
        master_grid.mapToWorld(xi, yj, wx, wy);
        double dx = wx - s.cx, dy = wy - s.cy;
        double d2 = dx * dx + dy * dy;
        if (d2 > r2) continue;

        double falloff = 1.0 - std::sqrt(d2) / r;          // 1 at center -> 0 at edge
        double cell_cost_d = k_o11y_cost_ * s.risk * falloff;
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
    ++painted_tracks;
  }

  auto node = node_.lock();
  if (node) {
    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
      "\033[35m[O11yRiskLayer] active tracks: %zu  painted: %zu\033[0m",
      tracks_.size(), painted_tracks);
  }
}

void O11yRiskLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  tracks_.clear();
  have_bounds_ = false;
}

}  // namespace nav2_o11y_risk_layer