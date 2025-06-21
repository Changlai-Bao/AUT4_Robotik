/**
 * @file wall_follower.cpp      
 * @author Prof. Dr. Christian Pfitzner (christian.pfitzner@th-nuernberg.de)
 * @brief 
 * @version 0.1
 * @date 2024-06-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */


// ros specific includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"



// This class subscribes to the lidar topic and publishes velocity commands to the cmd_vel topic

/**
 * @class WallFollowNode
 *
 * @brief A class that subscribes to the lidar topic and publishes velocity commands to the cmd_vel topic, 
 *        implementing a simple wall-following behavior.
 * 
 */
class WallFollowNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Wall Follow Node object
     * 
     */
    WallFollowNode() : Node("wall_follow_node")
    {
        // Create a subscriber to the lidar topic
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                            "/robot1/laser",                                    // topic name                   
                            10,                                                 // queue size
                            std::bind(&WallFollowNode::lidarCallback,           // callback function    
                            this, 
                            std::placeholders::_1)
                            );

        // Create a publisher for the cmd_vel topic
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                            "/robot1/cmd_vel",                                     
                            10);
    }

    /**
     * @brief Destroy the Wall Follow Node object
     * 
     */
    ~WallFollowNode()
    {
        // Do some cleanup if needed
    }

private:
    /**
     * @brief Callback function for the lidar subscriber
     * 
     * @param msg The lidar message
     */
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Implement wall-following behavior here
       




        

        // Implement a path controller based on the distance to the wall. 
        // A suggestion is a PI controller. 








        // Create a Twist message with the desired velocity
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x  = 0.5;  // Example: set linear velocity to 0.5 m/s
        cmd_vel_msg.angular.z = 0.1;  // Example: set angular velocity to 0.1 rad/s

        // Publish the cmd_vel message
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_publisher_;
};




/**
 * @brief 
 * 
 * @param argc  number of arguments provided from the command line
 * @param argv  array of arguments provided from the command line
 * @return int  0
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFollowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}