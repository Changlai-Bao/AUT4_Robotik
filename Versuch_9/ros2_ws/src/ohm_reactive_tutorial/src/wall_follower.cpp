/**
 * @file optimized_wall_follower.cpp
 * @author Prof. Dr. Christian Pfitzner (christian.pfitzner@th-nuernberg.de) / Gemini
 * @brief Optimierter Wall-Following Algorithmus, der proaktiv Hindernisse sucht.
 * @version 0.5 (Fehlerbehebung)
 * @date 2024-06-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <algorithm>
#include <limits>

/**
 * @class OptimizedWallFollowNode
 * @brief Ein optimierter Wall-Follower, der proaktiv das nächste Hindernis ansteuert,
 * anstatt blind nach einer Wand zu suchen.
 */
class OptimizedWallFollowNode : public rclcpp::Node
{
private:
    // Definition der Roboterzustände für eine klarere Logik
    enum class RobotState
    {
        SEEKING_OBSTACLE,
        WALL_FOLLOWING,
        AVOIDING_OBSTACLE
    };

    // Member-Variablen
    // KORREKTUR: Der korrekte Nachrichtentyp ist 'LaserScan' (CamelCase) und nicht 'Laser_scan'.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    float integral_error_;
    rclcpp::Time previous_time_;
    RobotState current_state_; // Variable zum Speichern des aktuellen Zustands

public:
    OptimizedWallFollowNode() : Node("wall_follow_node")
    {
        // Subscriber für Lidar-Daten
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot1/laser", 10,
            std::bind(&OptimizedWallFollowNode::lidarCallback, this, std::placeholders::_1));

        // Publisher für Geschwindigkeitsbefehle
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        // Initialisierung
        integral_error_ = 0.0f;
        previous_time_ = this->now();
        current_state_ = RobotState::SEEKING_OBSTACLE; // Start im optimierten Suchmodus

        RCLCPP_INFO(this->get_logger(), "Optimierter Wall-Follower gestartet. Suche nächstes Hindernis.");
    }

private:
    /**
     * @brief Berechnet den Winkel zur Wand (vereinfacht und stabiler)
     */
    float calculateWallAngle(const std::vector<float> &ranges,
                             float angle_min, float angle_increment,
                             float range_min, float range_max)
    {
        // Fixe Punkte für rechte Wand
        float point1_angle = -M_PI / 2;       // -90°
        float point2_angle = -M_PI / 2 + 0.5; // -60°

        int idx1 = static_cast<int>((point1_angle - angle_min) / angle_increment);
        int idx2 = static_cast<int>((point2_angle - angle_min) / angle_increment);

        if (idx1 < 0 || idx1 >= static_cast<int>(ranges.size()) ||
            idx2 < 0 || idx2 >= static_cast<int>(ranges.size()))
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        float range1 = ranges[idx1];
        float range2 = ranges[idx2];

        if (range1 < range_min || range1 > range_max || std::isnan(range1) || std::isinf(range1) ||
            range2 < range_min || range2 > range_max || std::isnan(range2) || std::isinf(range2))
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        // Kartesische Koordinaten
        float x1 = range1 * cos(point1_angle);
        float y1 = range1 * sin(point1_angle);
        float x2 = range2 * cos(point2_angle);
        float y2 = range2 * sin(point2_angle);

        // Einheitsvektor und Wandvektor
        float vector_a_x = 1.0f;
        float vector_a_y = 0.0f;
        float vector_b_x = x2 - x1;
        float vector_b_y = y2 - y1;

        float magnitude_b = sqrt(vector_b_x * vector_b_x + vector_b_y * vector_b_y);
        if (magnitude_b < 1e-6)
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        float dot_product = vector_a_x * vector_b_x + vector_a_y * vector_b_y;
        float cos_alpha = dot_product / magnitude_b;
        cos_alpha = std::max(-1.0f, std::min(1.0f, cos_alpha));

        return acos(cos_alpha);
    }

    /**
     * @brief Berechnet Wanddistanz
     */
    float calculateWallDistance(const std::vector<float> &ranges,
                                float angle_min, float angle_increment,
                                float range_min, float range_max)
    {
        float wall_angle = -M_PI / 2; // -90° für rechte Wand
        int idx = static_cast<int>((wall_angle - angle_min) / angle_increment);

        if (idx < 0 || idx >= static_cast<int>(ranges.size()))
        {
            return std::numeric_limits<float>::infinity();
        }

        float range = ranges[idx];
        if (range < range_min || range > range_max || std::isnan(range) || std::isinf(range))
        {
            return std::numeric_limits<float>::infinity();
        }

        return range;
    }

    /**
     * @brief Verbesserte PI-Regelung mit stabileren Parametern
     */
    float stablePiController(float angle_error, float dt)
    {
        float kp = 1.5f;
        float ki = 0.3f;

        integral_error_ += angle_error * dt;
        float max_integral = 1.0f;
        integral_error_ = std::max(-max_integral, std::min(max_integral, integral_error_));

        float control_output = kp * angle_error + ki * integral_error_;
        return std::max(-1.5f, std::min(1.5f, control_output));
    }

    /**
     * @brief Erkennt Hindernisse mit Hysterese
     */
    bool detectFrontObstacle(const std::vector<float> &ranges,
                             float angle_min, float angle_increment,
                             float range_min, float range_max)
    {
        std::vector<float> front_ranges = getRangesInSector(
            ranges, -M_PI / 6, M_PI / 6,
            angle_min, angle_increment, range_min, range_max);

        float min_front_distance = getMinDistance(front_ranges);

        static bool obstacle_detected = false;
        float threshold_detect = 1.8f;
        float threshold_clear = 2.2f;

        if (!obstacle_detected && min_front_distance < threshold_detect)
        {
            obstacle_detected = true;
        }
        else if (obstacle_detected && min_front_distance > threshold_clear)
        {
            obstacle_detected = false;
        }

        return obstacle_detected;
    }

    /**
     * @brief Hauptcallback mit Zustandslogik
     */
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        float dt = (current_time - previous_time_).seconds();
        if (dt <= 0.0f || dt > 0.1f)
            dt = 0.05f;
        previous_time_ = current_time;

        float wall_angle = calculateWallAngle(msg->ranges, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max);
        float wall_distance = calculateWallDistance(msg->ranges, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max);
        bool front_obstacle = detectFrontObstacle(msg->ranges, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max);

        // Zustandsübergänge bestimmen
        if (front_obstacle)
        {
            current_state_ = RobotState::AVOIDING_OBSTACLE;
        }
        else if (!std::isnan(wall_angle) && wall_distance < 4.0f)
        {
            current_state_ = RobotState::WALL_FOLLOWING;
        }
        else
        {
            current_state_ = RobotState::SEEKING_OBSTACLE;
        }

        float linear_vel = 0.0f;
        float angular_vel = 0.0f;

        switch (current_state_)
        {
        case RobotState::AVOIDING_OBSTACLE:
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Zustand: HINDERNISVERMEIDUNG");
            std::vector<float> left_ranges = getRangesInSector(msg->ranges, M_PI / 6, M_PI / 2, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max);
            std::vector<float> right_ranges = getRangesInSector(msg->ranges, -M_PI / 2, -M_PI / 6, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max);
            float left_clearance = getMinDistance(left_ranges);
            float right_clearance = getMinDistance(right_ranges);

            if (left_clearance > right_clearance && left_clearance > 1.0f)
            {
                linear_vel = 0.3f;
                angular_vel = 0.8f; // Links ausweichen
            }
            else if (right_clearance > 1.0f)
            {
                linear_vel = 0.3f;
                angular_vel = -0.8f; // Rechts ausweichen
            }
            else
            {
                linear_vel = -0.2f;
                angular_vel = 0.5f; // Notfall: Rückwärts
            }
            integral_error_ = 0.0f;
            break;
        }

        case RobotState::WALL_FOLLOWING:
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Zustand: WANDVERFOLGUNG - Winkel: %.2f°, Distanz: %.2fm", wall_angle * 180.0 / M_PI, wall_distance);
            float desired_wall_distance = 1.3f;
            float angle_error = wall_angle - 0.0f; // Gewünschter Winkel ist 0

            float distance_factor = 1.0f;
            if (wall_distance < desired_wall_distance - 0.3f)
                distance_factor = 1.2f;
            else if (wall_distance > desired_wall_distance + 0.3f)
                distance_factor = 0.8f;

            angular_vel = -stablePiController(angle_error, dt) * distance_factor;
            linear_vel = 0.6f; // Etwas schneller auf geraden Strecken

            if (std::abs(angular_vel) > 0.4f)
            {
                linear_vel *= 0.7f;
            }
            break;
        }

        case RobotState::SEEKING_OBSTACLE:
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Zustand: HINDERNISSUCHE");

            // *** OPTIMIERTE LOGIK START ***
            float min_range = std::numeric_limits<float>::infinity();
            int min_index = -1;

            // Finde den nächsten Punkt im gesamten Scanbereich
            for (size_t i = 0; i < msg->ranges.size(); ++i)
            {
                float range = msg->ranges[i];
                if (range >= msg->range_min && range < min_range)
                {
                    min_range = range;
                    min_index = i;
                }
            }

            if (min_index != -1)
            {
                // Berechne den Winkel zum nächsten Punkt
                float target_angle = msg->angle_min + min_index * msg->angle_increment;

                // Steuere auf das Hindernis zu
                linear_vel = 0.4f;                 // Zielgerichtet vorwärts fahren
                angular_vel = 0.8f * target_angle; // Proportionale Regelung zur Ausrichtung

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Nächstes Hindernis bei %.2fm und %.2f rad gefunden. Steuere darauf zu.", min_range, target_angle);
            }
            else
            {
                // Fallback, falls keine Daten vorhanden sind (sehr unwahrscheinlich)
                linear_vel = 0.1f;
                angular_vel = -0.3f;
            }
            // *** OPTIMIERTE LOGIK ENDE ***

            integral_error_ = 0.0f;
            break;
        }
        }

        // Sicherheitsbegrenzungen
        linear_vel = std::max(-0.3f, std::min(linear_vel, 0.7f));
        angular_vel = std::max(-1.5f, std::min(angular_vel, 1.5f));

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_vel;
        cmd_vel_msg.angular.z = angular_vel;
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }

    /**
     * @brief Hilfsfunktionen
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
                if (range >= range_min && range <= range_max && !std::isnan(range) && !std::isinf(range))
                {
                    sector_ranges.push_back(range);
                }
            }
        }
        return sector_ranges;
    }

    float getMinDistance(const std::vector<float> &ranges)
    {
        if (ranges.empty())
        {
            return std::numeric_limits<float>::infinity();
        }
        return *std::min_element(ranges.begin(), ranges.end());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OptimizedWallFollowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}