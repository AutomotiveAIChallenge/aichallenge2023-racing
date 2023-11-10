#include "aichallenge_scoring_msgs/msg/score.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"

#include <cstdio>
#include <iostream>
#include <fstream>

class ScoringResult : public rclcpp::Node
{
public:
  ScoringResult() : Node("score_node") {
    // Subscribers
    score_subscriber = this->create_subscription<aichallenge_scoring_msgs::msg::Score>(
      "/aichallenge/score", 1, std::bind(&ScoringResult::scoreCallback, this, std::placeholders::_1));
    collision_subscriber = this->create_subscription<geometry_msgs::msg::Point>(
      "/aichallenge/collision", 1, std::bind(&ScoringResult::collisionCallback, this, std::placeholders::_1));
  }

private:

  void scoreCallback(const aichallenge_scoring_msgs::msg::Score& msg) {
    if (is_result_generated_)
      return;
    
    const auto has_finished =
      msg.is_timeout || msg.is_lap_completed || msg.should_terminate_simulation ||
      has_collided_;
    if (!has_finished)
      return;

    std::cout << "Scoring completed." << std::endl;
    writeResultJson(msg);

    is_result_generated_ = true;
  }

  void collisionCallback(const geometry_msgs::msg::Point& msg) {
    RCLCPP_INFO(this->get_logger(), "Collision occured at: (%f, %f, %f)", msg.x, msg.y, msg.z);

    has_collided_ = true;
  }

  float calculateDistanceScore(const aichallenge_scoring_msgs::msg::Score& score_msg) {
    return score_msg.distance_score;
  }

  void writeResultJson(const aichallenge_scoring_msgs::msg::Score& score_msg) {
    std::ofstream ofs("/aichallenge/result.json");
    const double lap_time = score_msg.lap_time + static_cast<double>(score_msg.num_outside_lane) * 10.0 + static_cast<double>(score_msg.num_collision) * 10.0;
    ofs << "{" << std::endl;
    ofs << "  \"rawLapTime\": " << score_msg.raw_lap_time << "," << std::endl;
    ofs << "  \"distanceScore\": " << calculateDistanceScore(score_msg) << "," << std::endl;
    ofs << "  \"Duration\": " << lap_time << "," << std::endl;
    ofs << std::boolalpha << "  \"isLapCompleted\": " << score_msg.is_lap_completed << "," << std::endl;
    ofs << std::boolalpha << "  \"isTimeout\": " << score_msg.is_timeout << "," << std::endl;
    ofs << std::boolalpha << "  \"numOutsideLane\": " << score_msg.num_outside_lane << "," << std::endl;
    ofs << std::boolalpha << "  \"numCollision\": " << score_msg.num_collision << "," << std::endl;
    ofs << "}" << std::endl;
    ofs.close();
  }

  // Subscribers
  rclcpp::Subscription<aichallenge_scoring_msgs::msg::Score>::SharedPtr score_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr collision_subscriber;

  rclcpp::TimerBase::SharedPtr timer;

  // AWSIM
  bool has_collided_ = false;

  // Internal states
  bool is_result_generated_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScoringResult>());
  rclcpp::shutdown();
  return 0;
}
