#include "self_driving_controller.hpp"

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_system_msgs/msg/autoware_state.hpp"

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cstdio>

SelfDrivingController::SelfDrivingController()
 : Node("self_driving_controller")
 , step_counter_(0)
{
  goal_pose_.position.x = declare_parameter<double>("initialgoal.position.x",0.0);
  goal_pose_.position.y = declare_parameter<double>("initialgoal.position.y",0.0);
  goal_pose_.position.z = declare_parameter<double>("initialgoal.position.z",0.0);
  goal_pose_.orientation.x = declare_parameter<double>("initialgoal.orientation.x",0.0);
  goal_pose_.orientation.y = declare_parameter<double>("initialgoal.orientation.y",0.0);
  goal_pose_.orientation.z = declare_parameter<double>("initialgoal.orientation.z",0.0);
  goal_pose_.orientation.w = declare_parameter<double>("initialgoal.orientation.w",0.0);
  goal_pos_publisher =
    this->create_publisher<PoseStamped>("output/goal", 1);

  // Subscribers
  state_subscriber = this->create_subscription<AutowareState>(
    "input/state", 1, std::bind(&SelfDrivingController::stateCallback, this, std::placeholders::_1));
}

geometry_msgs::msg::PoseStamped SelfDrivingController::createGoalPoseMessage()
{
  auto msg = PoseStamped();
  msg.header.frame_id = "map";
  msg.pose = goal_pose_;
  return msg;
}

void SelfDrivingController::stateCallback(const AutowareState& msg) {
  RCLCPP_INFO_STREAM(this->get_logger(),msg.state);
  using namespace std::chrono_literals;
  rclcpp::WallRate loop_rate(5000ms);
  loop_rate.sleep();
  switch (msg.state) {
    case AutowareState::INITIALIZING:
      if (step_counter_ >= 20)
        break;

      RCLCPP_INFO(this->get_logger(), "[AIChallengeSample]: Publishing goal pose.");
      goal_pos_publisher->publish(createGoalPoseMessage());

      ++step_counter_;
      break;
    default:
      break;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfDrivingController>());
  rclcpp::shutdown();
  return 0;
}
