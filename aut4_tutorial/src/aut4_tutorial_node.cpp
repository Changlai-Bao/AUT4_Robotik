#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/**
 * @brief Tutorial for the 3rd AUT4 lecture
 * @author Stefan May
 * @date 3. April 2025
 */
class AUT4TutorialNode : public rclcpp::Node
{
public:
  /**
   * Constructor
   * @param[in] ...
   */
  AUT4TutorialNode() : Node("aut4_tutorial_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    auto timer_callback =
      [this]() -> void {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 1.0;
        this->publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AUT4TutorialNode>());
  rclcpp::shutdown();
  return 0;
}