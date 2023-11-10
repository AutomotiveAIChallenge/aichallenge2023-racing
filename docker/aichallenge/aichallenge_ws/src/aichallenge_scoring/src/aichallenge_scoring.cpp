// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "aichallenge_scoring/aichallenge_scoring.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <rclcpp/rclcpp.hpp>

namespace aichallenge_scoring {
  class AIChallengeScoringNode::StopWatch {
  public:
    StopWatch(const rclcpp::Clock::SharedPtr& clock)
      : system_clock_(clock) {
      tic(default_name);
    }

    void tic(const std::string & name = default_name) { t_start_[name] = system_clock_->now(); }

    void tic(const char * name) { tic(std::string(name)); }

    double toc(const std::string & name, const bool reset = false)
    {
      const auto t_start = t_start_.at(name);
      const auto t_end = system_clock_->now();
      const auto duration = (t_end - t_start).nanoseconds();

      if (reset) {
        t_start_[name] = system_clock_->now();
      }

      const auto one_sec = rclcpp::Duration(1, 0).nanoseconds();

      return static_cast<double>(duration) / one_sec;
    }

    double toc(const char * name, const bool reset = false) { return toc(std::string(name), reset); }

    double toc(const bool reset = false) { return toc(default_name, reset); }

  private:
    using Time = rclcpp::Time;
    static constexpr const char * default_name{"__auto__"};

    rclcpp::Clock::SharedPtr system_clock_;

    std::unordered_map<std::string, Time> t_start_;
  };

  AIChallengeScoringNode::AIChallengeScoringNode(const rclcpp::NodeOptions & node_options) 
  : Node("aichallenge_scoring", node_options)
  , vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
  , stop_watch_ptr_(std::make_unique<StopWatch>(get_clock())) {
    using std::placeholders::_1;

    start_distance_ = declare_parameter<double>("start_distance");
    end_distance_ = declare_parameter<double>("end_distance");

    // Subscribers
    sub_odom_ = create_subscription<Odometry>("/localization/kinematic_state", rclcpp::QoS(1), std::bind(&AIChallengeScoringNode::onOdom, this, _1));
    sub_map_ = create_subscription<HADMapBin>("/map/vector_map", rclcpp::QoS{1}.transient_local(), std::bind(&AIChallengeScoringNode::onMap, this, _1));

    // Publishers
    pub_footprint_marker_ = create_publisher<visualization_msgs::msg::Marker>("output/footprint_marker", rclcpp::QoS(1));
    pub_score_ = create_publisher<aichallenge_scoring_msgs::msg::Score>("output/score", rclcpp::QoS{1}.transient_local());

    // Timer
    using std::chrono_literals::operator""ms;
    timer_ = rclcpp::create_timer(this, get_clock(), 1000ms, std::bind(&AIChallengeScoringNode::onTimer, this));

    distance_score_= 0.0;
    duration_ = 0.0;
    total_duration_ = 0.0;
    num_outside_lane_= 0;
    num_collision_ = 0;
    has_finished_ = false;
    is_lap_completed_ = false;
    is_doing_ = false;
    odometry_ = nullptr;
    stop_watch_ptr_->tic("duration");
  }

  void AIChallengeScoringNode::onOdom(const Odometry::SharedPtr msg) {
    odometry_ = msg;
  }

  void AIChallengeScoringNode::onMap(const HADMapBin::ConstSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "[AIChallengeScoringNode]: Start loading lanelet");
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
    RCLCPP_INFO(this->get_logger(), "[AIChallengeScoringNode]: Map is loaded");
  }

  bool AIChallengeScoringNode::isDataReady() {
    if (lanelet_map_ptr_ == nullptr) {
      RCLCPP_WARN(get_logger(), "Map not received yet");
      return false;
    }

    if (!odometry_) {
      RCLCPP_WARN(get_logger(), "Vehicle odom not received yet");
      return false;
    }

    return true;
  }

  void AIChallengeScoringNode::onTimer() {
    static const std::string total_duration_timer_name = "total_duration";

    if (!isDataReady()) {
      return;
    }

    // Get closest lanelet to current vehicle pose
    if (!lanelet::utils::query::getClosestLanelet(
        lanelet::utils::query::laneletLayer(lanelet_map_ptr_), 
        odometry_->pose.pose, &closest_lanelet_)) {
      RCLCPP_INFO(this->get_logger(), "No closest lanelet");
      return;
    }

    // Calculate the distance along the center line linestring to the vehicle position point
    lanelet::BasicPoint2d vehicle_pos(odometry_->pose.pose.position.x, odometry_->pose.pose.position.y);
    const auto arc_coordinates = lanelet::geometry::toArcCoordinates(lanelet::utils::to2D(closest_lanelet_.centerline().basicLineString()), vehicle_pos);

    // Retrieve distance_score
    distance_score_ = arc_coordinates.length;

    // Check start of self-driving
    if (!has_started_driving_) {
      RCLCPP_INFO(this->get_logger(), "Self-driving started");
      has_started_driving_ = true;
      stop_watch_ptr_->tic(total_duration_timer_name);
    }

    // Update total duration
    auto total_duration = 0.0;
    if (has_started_driving_) {
      total_duration = stop_watch_ptr_->toc(total_duration_timer_name, false);
    }
    const auto timeout_time = 10.0 * 60.0;
    auto is_timeout = total_duration > timeout_time;

    // Start of Racing: Start timer
    if (distance_score_ >= start_distance_ && distance_score_ <= end_distance_ && !is_doing_) {
      RCLCPP_INFO(this->get_logger(), "started racing");
      duration_ = stop_watch_ptr_->toc("duration", true);
      is_doing_ = true;
    }

    // End Racing: Stop timer
    if (distance_score_ > end_distance_ && is_doing_) {
      duration_ = stop_watch_ptr_->toc("duration", false);
      is_doing_ = false;
      has_finished_ = true;
    }

    // Prepare message to publish
    aichallenge_scoring_msgs::msg::Score score_msg;
    score_msg.distance_score = std::min(distance_score_, end_distance_);
    if (is_doing_) {
      score_msg.lap_time = stop_watch_ptr_->toc("duration", false);
    } else if (has_finished_) {
      score_msg.lap_time = duration_;
    }
    score_msg.is_lap_completed = distance_score_ >= end_distance_;
    score_msg.raw_lap_time = total_duration;
    score_msg.is_timeout = is_timeout;

    // Check if vehicle is inside lane
    createVehicleFootprint(vehicle_info_);
    const auto is_outside_lane = isOutsideLaneFromVehicleFootprint(closest_lanelet_);
    if(is_outside_lane){
      num_outside_lane_++;
    }

    // TODO add implementation of collision

    if(score_msg.num_outside_lane > 10 && score_msg.num_collision > 10){
      score_msg.should_terminate_simulation = true;
    } else{
      score_msg.should_terminate_simulation = false;
    }
    visualizeVehicleFootprint(is_outside_lane);

    pub_score_->publish(score_msg);
  }

  void AIChallengeScoringNode::createVehicleFootprint(const vehicle_info_util::VehicleInfo & vehicle_info) {
    // Create vehicle footprint
    const double x_front = vehicle_info.front_overhang_m + vehicle_info.wheel_base_m;
    const double x_rear = -(vehicle_info.rear_overhang_m);
    const double y_left = vehicle_info.wheel_tread_m / 2.0 + vehicle_info.left_overhang_m;
    const double y_right = -(vehicle_info.wheel_tread_m / 2.0 + vehicle_info.right_overhang_m);

    tier4_autoware_utils::LinearRing2d local_vehicle_footprint;
    local_vehicle_footprint.push_back(tier4_autoware_utils::Point2d{x_front, y_left});
    local_vehicle_footprint.push_back(tier4_autoware_utils::Point2d{x_front, y_right});
    local_vehicle_footprint.push_back(tier4_autoware_utils::Point2d{x_rear, y_right});
    local_vehicle_footprint.push_back(tier4_autoware_utils::Point2d{x_rear, y_left});
    local_vehicle_footprint.push_back(tier4_autoware_utils::Point2d{x_front, y_left});

    // Transform the local_vehicle_footprint to map-frame
    vehicle_footprint_ = tier4_autoware_utils::transformVector(local_vehicle_footprint, tier4_autoware_utils::pose2transform(odometry_->pose.pose));
  }

  bool AIChallengeScoringNode::isOutsideLaneFromVehicleFootprint(const lanelet::ConstLanelet & lanelet) {
    // Check if footprint is completely within the lane
    if (!boost::geometry::within(vehicle_footprint_, lanelet.polygon2d().basicPolygon())) {
      // Vehicle is not completely within the lane
      return true;
    }

    // Vehicle is completely within the lane
    return false;
  }

  void AIChallengeScoringNode::visualizeVehicleFootprint(bool is_outside_lane) {

    // Create marker for visualizing vehicle footprint
    visualization_msgs::msg::Marker marker;
    std_msgs::msg::ColorRGBA cl;
    
    if (is_outside_lane) {
      cl.r = 1.0;
      cl.g = 0.0;
      cl.b = 0.0;
      cl.a = 0.99;
    } else {
      cl.r = 0.0;
      cl.g = 1.0;
      cl.b = 0.0;
      cl.a = 0.99;
    }
    
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Time();
    marker.frame_locked = false;
    marker.id = 0;
    marker.ns = "aichallenge_scoring";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color = cl;

    for (const auto & point : vehicle_footprint_) {
      marker.points.push_back(tier4_autoware_utils::toMsg(point.to_3d()));
    }
    
    pub_footprint_marker_->publish(marker);
  }

} // namespace aichallenge_scoring

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(aichallenge_scoring::AIChallengeScoringNode)