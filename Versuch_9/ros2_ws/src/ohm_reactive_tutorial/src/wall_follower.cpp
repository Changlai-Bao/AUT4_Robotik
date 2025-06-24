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
#include <vector>


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

        // PI controller initialization
        integral_error_ = 0.0f;
        previous_time_ = this->now();
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
     * @brief Calculate wall angle using the formula: alpha = arccos(vector_a · vector_b / (|vector_a| * |vector_b|))
     * where vector_a is unit vector in x-direction and vector_b is vector between two wall points
     *
     * @param ranges The laser scan ranges
     * @param angle_min LaserScan angle_min
     * @param angle_increment LaserScan angle_increment
     * @param range_min Minimum valid range
     * @param range_max Maximum valid range
     * @return float Wall angle in radians, or NaN if calculation not possible
     */
    float calculateWallAngle(const std::vector<float> &ranges,
                             float angle_min, float angle_increment,
                             float range_min, float range_max)
    {
        // Find two points on the wall (right side for wall following)
        float point1_angle = -M_PI / 2;       // -90 degrees (right side)
        float point2_angle = -M_PI / 2 + 0.5; // -90 + 30 degrees

        // Convert angles to indices
        int idx1 = static_cast<int>((point1_angle - angle_min) / angle_increment);
        int idx2 = static_cast<int>((point2_angle - angle_min) / angle_increment);

        // Check if indices are valid
        if (idx1 < 0 || idx1 >= static_cast<int>(ranges.size()) ||
            idx2 < 0 || idx2 >= static_cast<int>(ranges.size()))
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        // Get ranges for both points
        float range1 = ranges[idx1];
        float range2 = ranges[idx2];

        // Check if ranges are valid
        if (range1 < range_min || range1 > range_max || std::isnan(range1) || std::isinf(range1) ||
            range2 < range_min || range2 > range_max || std::isnan(range2) || std::isinf(range2))
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        // Convert polar coordinates to Cartesian coordinates
        float x1 = range1 * cos(point1_angle);
        float y1 = range1 * sin(point1_angle);
        float x2 = range2 * cos(point2_angle);
        float y2 = range2 * sin(point2_angle);

        // Vector a: unit vector in x-direction
        float vector_a_x = 1.0f;
        float vector_a_y = 0.0f;

        // Vector b: vector between the two wall points
        float vector_b_x = x2 - x1;
        float vector_b_y = y2 - y1;

        // Calculate magnitudes
        float magnitude_a = sqrt(vector_a_x * vector_a_x + vector_a_y * vector_a_y); // = 1.0 for unit vector
        float magnitude_b = sqrt(vector_b_x * vector_b_x + vector_b_y * vector_b_y);

        // Check if magnitude_b is zero (points are the same)
        if (magnitude_b < 1e-6)
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        // Calculate dot product: vector_a · vector_b
        float dot_product = vector_a_x * vector_b_x + vector_a_y * vector_b_y;

        // Calculate alpha using the formula: alpha = arccos(dot_product / (magnitude_a * magnitude_b))
        float cos_alpha = dot_product / (magnitude_a * magnitude_b);

        // Clamp to valid range for arccos [-1, 1]
        cos_alpha = std::max(-1.0f, std::min(1.0f, cos_alpha));

        float alpha = acos(cos_alpha);

        return alpha;
    }
    /**
     * @brief Calculate distance to wall using two points method
     *
     * @param ranges The laser scan ranges
     * @param angle_min LaserScan angle_min
     * @param angle_increment LaserScan angle_increment
     * @param range_min Minimum valid range
     * @param range_max Maximum valid range
     * @return float Distance to wall, or infinity if calculation not possible
     */
    float calculateWallDistance(const std::vector<float> &ranges,
                                float angle_min, float angle_increment,
                                float range_min, float range_max)
    {
        // Use point at -90 degrees for distance calculation
        float wall_angle = -M_PI / 2; // -90 degrees (right side)

        // Convert angle to index
        int idx = static_cast<int>((wall_angle - angle_min) / angle_increment);

        // Check if index is valid
        if (idx < 0 || idx >= static_cast<int>(ranges.size()))
        {
            return std::numeric_limits<float>::infinity();
        }

        float range = ranges[idx];

        // Check if range is valid
        if (range < range_min || range > range_max || std::isnan(range) || std::isinf(range))
        {
            return std::numeric_limits<float>::infinity();
        }

        return range;
    }
    /**
     * @brief Get ranges in a specific angular sector
     *
     * @param ranges The laser scan ranges
     * @param angle_min Start angle (radians)
     * @param angle_max End angle (radians)
     * @param scan_angle_min LaserScan angle_min
     * @param scan_angle_increment LaserScan angle_increment
     * @param range_min Minimum valid range from LaserScan
     * @param range_max Maximum valid range from LaserScan
     * @return std::vector<float> Ranges in the specified sector
     */
    std::vector<float> getRangesInSector(const std::vector<float> &ranges,
                                         float angle_min, float angle_max,
                                         float scan_angle_min, float scan_angle_increment,
                                         float range_min, float range_max)
    {
        std::vector<float> sector_ranges;

        for (size_t i = 0; i < ranges.size(); ++i)
        {
            float current_angle = scan_angle_min + i * scan_angle_increment;

            if (current_angle >= angle_min && current_angle <= angle_max)
            {
                float range = ranges[i];
                // Filter out invalid ranges using LaserScan's limits
                if (range >= range_min && range <= range_max &&
                    !std::isnan(range) && !std::isinf(range))
                {
                    sector_ranges.push_back(range);
                }
            }
        }

        return sector_ranges;
    }

    /**
     * @brief Calculate minimum distance in a range vector
     *
     * @param ranges Vector of ranges
     * @return float Minimum distance, or infinity if no valid ranges
     */
    float getMinDistance(const std::vector<float> &ranges)
    {
        if (ranges.empty())
        {
            return std::numeric_limits<float>::infinity();
        }

        return *std::min_element(ranges.begin(), ranges.end());
    }
     /**
     * @brief PI controller for wall following angle
     *
     * @param angle_error Current angle error (radians)
     * @param dt Time step (seconds)
     * @return float Control output (angular velocity)
     */
    float piController(float angle_error, float dt)
    {
        // PI controller parameters
        float kp = 2.0f; // Proportional gain default 2.0f
        float ki = 0.5f; // Integral gain default 0.5f

        // Update integral term
        integral_error_ += angle_error * dt;

        // Anti-windup: limit integral term
        //float max_integral = 1.0f;
        //integral_error_ = std::max(-max_integral, std::min(max_integral, integral_error_));

        // Calculate PI control output
        float control_output = kp * angle_error + ki * integral_error_;

        return control_output;
    }
    /**
     * @brief Callback function for the lidar subscriber
     * 
     * @param msg The lidar message
     */
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Implement wall-following behavior here

        // Calculate wall angle using the formula: alpha = arccos(vector_a · vector_b / (|vector_a| * |vector_b|))
        float wall_angle = calculateWallAngle(msg->ranges, msg->angle_min, msg->angle_increment,
                                            msg->range_min, msg->range_max);
        
        // Calculate distance to wall
        float wall_distance = calculateWallDistance(msg->ranges, msg->angle_min, msg->angle_increment,
            msg->range_min, msg->range_max);
        
        // Get front ranges for obstacle detection
        std::vector<float> front_ranges = getRangesInSector(
            msg->ranges, -M_PI/6, M_PI/6,   // -30° to +30° (front)
            msg->angle_min, msg->angle_increment,
            msg->range_min, msg->range_max
        );
        float min_front_distance = getMinDistance(front_ranges);

        

        // Implement a path controller based on the distance to the wall.  
        float linear_vel = 0.5;  // Example: set linear velocity to 0.5 m/s
        float angular_vel = 0.1;  // Example: set angular velocity to 0.1 rad/s (turns left)

        
         // Wall following parameters
         float desired_wall_distance = 1.5f;  // 1.0 meters from wall
         float desired_wall_angle = 0.0f;     // 0.0 radians (parallel to wall)

         // Calculate current time and time step
        rclcpp::Time current_time = this->now();
        float dt = (current_time - previous_time_).seconds();
        previous_time_ = current_time;
        float angle_error = 0;
        
        // Avoid division by zero or very small dt
        if (dt <= 0.0f || dt > 0.1f) {
            dt = 0.05f; // Default 20Hz update rate
        }

        // Log calculated values for debugging
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Wall angle: %.3f rad (%.1f°), Wall distance: %.2fm, Front min: %.2fm",
            wall_angle, wall_angle * 180.0 / M_PI, wall_distance, min_front_distance);

        if (!std::isnan(wall_angle) && !std::isinf(wall_distance))
        {
            // Calculate errors
            angle_error = wall_angle - desired_wall_angle;

            // A suggestion is a PI controller.
            angular_vel = -piController(angle_error, dt);
            
            // Forward velocity based on front obstacle distance
            if (min_front_distance < desired_wall_distance)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Minimun Distance exceeded. Slowing down robot...");
                linear_vel -= 0.25f; // Slowing down
                // A suggestion is a PI controller.
                angular_vel = piController(angle_error, dt);
            }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Angular velocity: %.2f , Linear velocity: %.2f, Anuglar error: %.2f, dt: %.3fs, Integral: %.3f",
            angular_vel, linear_vel, angle_error, dt, integral_error_);
        // Reset integral when no wall is detected
        integral_error_ = 0.0f;




        // Create a Twist message with the desired velocity
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x  = linear_vel;
        cmd_vel_msg.angular.z = angular_vel;

        // Publish the cmd_vel message
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_publisher_;
    // PI controller variables
    float integral_error_;
    rclcpp::Time previous_time_;
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