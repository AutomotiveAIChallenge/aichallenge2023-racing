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

#ifndef AICHALLENGE_SCORING__AICHALLENGE_SCORING_HPP_
#define AICHALLENGE_SCORING__AICHALLENGE_SCORING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <aichallenge_scoring_msgs/msg/score.hpp>

namespace aichallenge_scoring {

  using nav_msgs::msg::Odometry;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using autoware_auto_perception_msgs::msg::DetectedObjects;

  class AIChallengeScoringNode : public rclcpp::Node {
    public:
      explicit AIChallengeScoringNode(const rclcpp::NodeOptions & options);

      bool isDataReady();
      void onTimer();
      void onOdom(const Odometry::SharedPtr msg);
      void onObjects(const DetectedObjects::SharedPtr msg);
      void onMap(const HADMapBin::ConstSharedPtr msg);
      void createVehicleFootprint(const vehicle_info_util::VehicleInfo & vehicle_info);
      bool isOutsideLaneFromVehicleFootprint(const lanelet::ConstLanelet & lanelet);
      bool isColliedWithNPC();
      void visualizeVehicleFootprint(bool is_outside_lane);


    private:
      class StopWatch;

      // Subscribers
      rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
      rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
      rclcpp::Subscription<DetectedObjects>::SharedPtr sub_objects_;

      // Publishers
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_footprint_marker_;
      rclcpp::Publisher<aichallenge_scoring_msgs::msg::Score>::SharedPtr pub_score_;

      vehicle_info_util::VehicleInfo vehicle_info_{};

      bool is_lap_completed_;
      double distance_score_;
      double start_distance_;
      double end_distance_;
      double duration_;
      double total_duration_;
      bool has_started_driving_;
      bool has_exceeded_speed_limit_;
      bool is_doing_;
      bool has_finished_;
      int num_outside_lane_;
      int num_collision_;

      rclcpp::TimerBase::SharedPtr timer_;
      Odometry::ConstSharedPtr odometry_;
      DetectedObjects::ConstSharedPtr objcts_;
      lanelet::ConstLanelet closest_lanelet_;
      lanelet::LaneletMapPtr lanelet_map_ptr_;
      std::unique_ptr<StopWatch> stop_watch_ptr_;
      tier4_autoware_utils::LinearRing2d vehicle_footprint_;
  };
} // namespace aichallenge_scoring

#endif // AICHALLENGE_SCORING__AICHALLENGE_SCORING_HPP_