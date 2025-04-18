#include "rclcpp/rclcpp.hpp"
#include "SerialPort.h"

class SerialNode : public rclcpp::Node
{
public:
  SerialNode() : Node("st_vl6180_node")
  {
    const char comPort[] = "/dev/ttyACM0";
    const speed_t baud = B115200;

    serial_port_ = std::make_unique<SerialPort>(comPort, baud);
    serial_port_->flush();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SerialNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Serial node started on port %s", comPort);
  }

private:
  void timer_callback()
  {
    char buf[2];
    if (serial_port_->receive(buf, 2))
    {
      RCLCPP_INFO(this->get_logger(), "Byte1: %d, Byte2: %d", buf[0], buf[1]);
    }
  }

  std::unique_ptr<SerialPort> serial_port_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}