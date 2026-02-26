#include <gnn_objects_layer/gnn_objects_layer.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/costmap_math.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

PLUGINLIB_EXPORT_CLASS(gnn_objects_layer::GNNObjectsLayer, nav2_costmap_2d::CostmapLayer)

namespace gnn_objects_layer
{

void GNNObjectsLayer::onInitialize()
{
    ros_node_ = rclcpp::Node::make_shared("gnn_objects_layer");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    declareParameter("topic", rclcpp::ParameterValue("/tracked_polygons"));
    if (!ros_node_->get_parameter("topic", topic_) || topic_.empty()) {
        topic_ = "/tracked_polygons";
        RCLCPP_WARN(ros_node_->get_logger(), "⚠️ Parameter 'topic' not set. Using default: %s", topic_.c_str());
    }

    declareParameter("target_frame", rclcpp::ParameterValue("map"));
    if (!ros_node_->get_parameter("target_frame", target_frame_) || target_frame_.empty()) {
        target_frame_ = "map";
        RCLCPP_WARN(ros_node_->get_logger(), "⚠️ Parameter 'target_frame' not set. Using default: %s", target_frame_.c_str());
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = ros_node_->create_subscription<gnn_interfaces::msg::TrackedPolygon>(
        topic_, qos,
        std::bind(&GNNObjectsLayer::polygonCallback, this, std::placeholders::_1));

    current_ = true;

    executor_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(ros_node_);
        exec.spin();
    });

    RCLCPP_INFO(ros_node_->get_logger(), "✅ GNNObjectsLayer initialized and subscribed to %s", topic_.c_str());
}

void GNNObjectsLayer::polygonCallback(const gnn_interfaces::msg::TrackedPolygon::SharedPtr msg)
{
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));

        gnn_interfaces::msg::TrackedPolygon transformed = *msg;
        for (auto & pt : transformed.polygon.points) {
            tf2::doTransform(pt, pt, transform);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        tracked_polygons_.push_back({transformed, ros_node_->now()});

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(ros_node_->get_logger(), "TF error in polygonCallback: %s", ex.what());
    }
}

void GNNObjectsLayer::updateBounds(
    double, double, double,
    double* min_x, double* min_y, double* max_x, double* max_y)
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto & tracked : tracked_polygons_) {
        const auto& poly = tracked.polygon.polygon;
        for (const auto & pt : poly.points) {
            *min_x = std::min(*min_x, static_cast<double>(pt.x));
            *min_y = std::min(*min_y, static_cast<double>(pt.y));
            *max_x = std::max(*max_x, static_cast<double>(pt.x));
            *max_y = std::max(*max_y, static_cast<double>(pt.y));
        }
    }
}

void GNNObjectsLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& /*master_grid*/,
    int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
    // To be implemented in Phase 2
}

void GNNObjectsLayer::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    tracked_polygons_.clear();
    current_ = true;
}

bool GNNObjectsLayer::isClearable()
{
    return true;
}

}  // namespace gnn_objects_layer
