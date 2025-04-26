#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "SerialPort.h"

using namespace std::chrono_literals;

/**
 * @class XNucleoIKS01A3Node
 * @brief Publishes IMU data from a STM IMU
 * @date 26.04.2025
 * @author Changlai Bao
 */
class XNucleoIKS01A3Node : public rclcpp::Node
{
public:
  XNucleoIKS01A3Node() : Node("xnucleo_iks03a1_node")
  {
    _state = 0;
    _publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);
    _serial = new SerialPort("/dev/ttyACM0", B115200);
    _serial->flush();
    _timer = this->create_wall_timer(20ms, std::bind(&XNucleoIKS01A3Node::timer_callback, this));
  }

private:
  void timer_callback()
  {

    switch (_state)
    {
    // A start byte is used to find the beginning of a message. It is not very robust, but sufficient for testing.
    // TODO: find a better protocol solution
    case 0:
      if (_serial->receive((char *)_buf, 1))
      {
        if (_buf[0] == 0xFF)
        {
          _state = 1;
        }
        else
        {
          std::cout << "no sync, try to find start byte: " << (int)_buf[0] << std::endl;
        }
      }
      break;
    case 1:
      // 28 Bytes contain 3x4 Bytes for accelerometer data (3 float values)
      //                  3x4 Bytes for gyroscope data (3 float values)
      //                  1x4 Bytes for temperature data (1 float value)
      if (_serial->receive((char *)_buf, 28))
      {
        rclcpp::Time time = this->now();

        // Interprete IMU data (float values are directly casted to a character stream).
        float *fbuf = (float *)(&_buf[0]);

        //---------------------
        // --- Accelerometer --
        //---------------------
        float acc[3];
        acc[0] = fbuf[0] / 100.f;
        acc[1] = fbuf[1] / 100.f;
        acc[2] = -fbuf[2] / 100.f;

        //---------------------
        //----- Gyroscope -----
        //---------------------
        float gyr[3];
        gyr[0] = -fbuf[3] / 1000.f * M_PI / 180.f;
        gyr[1] = -fbuf[4] / 1000.f * M_PI / 180.f;
        gyr[2] = fbuf[5] / 1000.f * M_PI / 180.f;

        float temp = fbuf[6];

        // Publish IMU message
        sensor_msgs::msg::Imu msg;
        msg.header.frame_id = "odom";
        msg.header.stamp = time;
        msg.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        msg.linear_acceleration.x = acc[0];
        msg.linear_acceleration.y = acc[1];
        msg.linear_acceleration.z = acc[2];
        msg.linear_acceleration_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        msg.angular_velocity.x = gyr[0];
        msg.angular_velocity.y = gyr[1];
        msg.angular_velocity.z = gyr[2];
        msg.angular_velocity_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        _publisher->publish(msg);

        // Temperature data is not used yet, so, just display it.
        std::cout << "Temperature: " << temp << std::endl;

        _state = 0;
      }
      break;
    }
  }
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisher;
  SerialPort *_serial;
  unsigned char _buf[29];
  int _state;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XNucleoIKS01A3Node>());
  rclcpp::shutdown();
  return 0;
}