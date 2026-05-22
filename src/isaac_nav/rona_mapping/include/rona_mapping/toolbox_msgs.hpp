/*
 * rona_mapping
 * Copyright (c) 2019, Steve Macenski
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#ifndef SLAM_TOOLBOX__TOOLBOX_MSGS_HPP_
#define SLAM_TOOLBOX__TOOLBOX_MSGS_HPP_

#include "nav_msgs/msg/map_meta_data.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

#include "rona_mapping/srv/pause.hpp"
#include "rona_mapping/srv/reset.hpp"
#include "rona_mapping/srv/clear_queue.hpp"
#include "rona_mapping/srv/toggle_interactive.hpp"
#include "rona_mapping/srv/clear.hpp"
#include "rona_mapping/srv/save_map.hpp"
#include "rona_mapping/srv/loop_closure.hpp"
#include "rona_mapping/srv/serialize_pose_graph.hpp"
#include "rona_mapping/srv/deserialize_pose_graph.hpp"
#include "rona_mapping/srv/merge_maps.hpp"
#include "rona_mapping/srv/add_submap.hpp"
#include "rona_mapping/msg/localized_laser_scan.hpp"

#endif  // SLAM_TOOLBOX__TOOLBOX_MSGS_HPP_
