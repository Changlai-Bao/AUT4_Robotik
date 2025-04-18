#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief turtlebot_follower_node
 * @author Changlai Bao
 * @date 08. April 2025
 */
class TurtleNode : public rclcpp::Node
{
public:
  /**
   * Constructor
   * @param[in] ...
   */
  TurtleNode() : Node("turtlebot_follower_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&TurtleNode::timer_callback, this));
    subPose1_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleNode::pose1_callback, this, _1));
    subPose2_ = this->create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10, std::bind(&TurtleNode::pose2_callback, this, _1));
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::Twist msg;
    double dx = Pose1_.x - Pose2_.x;
    double dy = Pose1_.y - Pose2_.y;
    double distance = sqrt(dx * dx + dy * dy);

    const double distance_threshold = 0.1;

    if (distance > distance_threshold)
    {
      double phi = atan2(dy, dx);
      double psi = phi - Pose2_.theta;

      if (psi > M_PI)
      {
        psi -= 2 * M_PI;
      }
      if (psi < -M_PI)
      {
        psi += 2 * M_PI;
      }

      const double max_linear_speed = 2.0;
      msg.linear.x = std::min(distance, max_linear_speed);
      msg.angular.z = psi;
    }
    else
    {

      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
    }

    this->publisher_->publish(msg);
  }

  void pose1_callback(const turtlesim::msg::Pose &Pose)
  {
    Pose1_ = Pose;
  }

  void pose2_callback(const turtlesim::msg::Pose &Pose)
  {
    Pose2_ = Pose;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subPose1_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subPose2_;

  turtlesim::msg::Pose Pose1_;
  turtlesim::msg::Pose Pose2_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleNode>());
  rclcpp::shutdown();
  return 0;
}