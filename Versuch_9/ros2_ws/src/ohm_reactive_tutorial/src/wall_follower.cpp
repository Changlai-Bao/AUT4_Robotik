/**
 * @file improved_wall_follower.cpp
 * @author Prof. Dr. Christian Pfitzner / AI Assistant
 * @brief Verbesserte Wall-Following Implementierung mit stabilerer Geradeausfahrt
 * @version 1.0
 * @date 2024-06-24
 *
 * @copyright Copyright (c) 2024
 *
 * Verbesserungen:
 * - Stabilere PI-Regelung mit angepassten Parametern
 * - Deadband für minimale Korrekturen
 * - Glättung der Winkelberechnung
 * - Geschwindigkeitsanpassung bei starken Lenkbewegungen
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <algorithm>
#include <limits>
#include <deque>

/**
 * @class StabilizedWallFollowNode
 * @brief Stabilisierte Wall-Follower Implementierung mit reduziertem Oszillieren
 */
class StabilizedWallFollowNode : public rclcpp::Node
{
private:
    // Roboterzustände für strukturierte Logik
    enum class RobotState
    {
        SEEKING_OBSTACLE,    // Hindernissuche
        WALL_FOLLOWING,      // Wandverfolgung
        AVOIDING_OBSTACLE    // Hindernisausweichung
    };

    // ROS2 Subscriber und Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Regelungsvariablen
    float integral_error_;
    float previous_error_;           // Für D-Anteil (falls benötigt)
    rclcpp::Time previous_time_;
    RobotState current_state_;

    // Glättungsfilter für Winkelberechnung
    std::deque<float> angle_history_;
    static const size_t ANGLE_FILTER_SIZE = 5;

    // Konfigurierbare Parameter
    const float DESIRED_WALL_DISTANCE = 1.3f;     // Gewünschter Wandabstand
    const float DEADBAND_THRESHOLD = 0.05f;       // Totband für kleine Korrekturen
    const float MAX_INTEGRAL = 0.8f;              // Begrenzung des I-Anteils
    
    // Verbesserte PI-Parameter für stabilere Regelung
    const float KP = 1.2f;                        // Reduzierter P-Anteil
    const float KI = 0.2f;                        // Reduzierter I-Anteil
    const float KD = 0.1f;                        // Kleiner D-Anteil für Dämpfung

public:
    StabilizedWallFollowNode() : Node("wall_follow_node")
    {
        // Subscriber für Lidar-Daten
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot1/laser", 10,
            std::bind(&StabilizedWallFollowNode::lidarCallback, this, std::placeholders::_1));

        // Publisher für Geschwindigkeitsbefehle
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        // Initialisierung
        integral_error_ = 0.0f;
        previous_error_ = 0.0f;
        previous_time_ = this->now();
        current_state_ = RobotState::SEEKING_OBSTACLE;

        RCLCPP_INFO(this->get_logger(), "Stabilisierter Wall-Follower gestartet.");
    }

private:
    /**
     * @brief Berechnet den geglätteten Winkel zur Wand
     */
    float calculateSmoothedWallAngle(const std::vector<float> &ranges,
                                    float angle_min, float angle_increment,
                                    float range_min, float range_max)
    {
        // Zwei Punkte für Wandwinkelberechnung (rechte Wand)
        float point1_angle = -M_PI / 2;            // -90°
        float point2_angle = -M_PI / 2 + 0.4f;     // Etwas kleinerer Winkel für Stabilität

        int idx1 = static_cast<int>((point1_angle - angle_min) / angle_increment);
        int idx2 = static_cast<int>((point2_angle - angle_min) / angle_increment);

        // Gültigkeitsprüfung der Indizes
        if (idx1 < 0 || idx1 >= static_cast<int>(ranges.size()) ||
            idx2 < 0 || idx2 >= static_cast<int>(ranges.size()))
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        float range1 = ranges[idx1];
        float range2 = ranges[idx2];

        // Gültigkeitsprüfung der Messwerte
        if (range1 < range_min || range1 > range_max || std::isnan(range1) || std::isinf(range1) ||
            range2 < range_min || range2 > range_max || std::isnan(range2) || std::isinf(range2))
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        // Kartesische Koordinaten berechnen
        float x1 = range1 * cos(point1_angle);
        float y1 = range1 * sin(point1_angle);
        float x2 = range2 * cos(point2_angle);
        float y2 = range2 * sin(point2_angle);

        // Wandwinkel berechnen
        float vector_b_x = x2 - x1;
        float vector_b_y = y2 - y1;
        float magnitude_b = sqrt(vector_b_x * vector_b_x + vector_b_y * vector_b_y);
        
        if (magnitude_b < 1e-6)
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        float cos_alpha = vector_b_x / magnitude_b;
        cos_alpha = std::max(-1.0f, std::min(1.0f, cos_alpha));
        float raw_angle = acos(cos_alpha);

        // Winkel zur Historie hinzufügen und glätten
        angle_history_.push_back(raw_angle);
        if (angle_history_.size() > ANGLE_FILTER_SIZE)
        {
            angle_history_.pop_front();
        }

        // Mittelwert der letzten Winkel berechnen
        float smoothed_angle = 0.0f;
        for (float angle : angle_history_)
        {
            smoothed_angle += angle;
        }
        return smoothed_angle / angle_history_.size();
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
     * @brief Verbesserte PID-Regelung mit Deadband und Anti-Windup
     */
    float stabilizedPidController(float angle_error, float dt)
    {
        // Deadband: Ignoriere sehr kleine Fehler
        if (std::abs(angle_error) < DEADBAND_THRESHOLD)
        {
            angle_error = 0.0f;
        }

        // Integralfehler aktualisieren
        integral_error_ += angle_error * dt;
        
        // Anti-Windup: Begrenze Integralfehler
        integral_error_ = std::max(-MAX_INTEGRAL, std::min(MAX_INTEGRAL, integral_error_));

        // Differenzfehler berechnen
        float derivative_error = (angle_error - previous_error_) / dt;
        previous_error_ = angle_error;

        // PID-Regelung
        float control_output = KP * angle_error + KI * integral_error_ + KD * derivative_error;
        
        // Ausgangsbegrenzung
        return std::max(-1.2f, std::min(1.2f, control_output));
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
     * @brief Hauptcallback mit verbesserter Zustandslogik
     */
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        float dt = (current_time - previous_time_).seconds();
        if (dt <= 0.0f || dt > 0.1f) dt = 0.05f;
        previous_time_ = current_time;

        float wall_angle = calculateSmoothedWallAngle(msg->ranges, msg->angle_min, 
                                                     msg->angle_increment, 
                                                     msg->range_min, msg->range_max);
        float wall_distance = calculateWallDistance(msg->ranges, msg->angle_min, 
                                                   msg->angle_increment, 
                                                   msg->range_min, msg->range_max);
        bool front_obstacle = detectFrontObstacle(msg->ranges, msg->angle_min, 
                                                 msg->angle_increment, 
                                                 msg->range_min, msg->range_max);

        // Zustandsübergänge
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
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "Zustand: HINDERNISAUSWEICHUNG");
            
            std::vector<float> left_ranges = getRangesInSector(msg->ranges, M_PI / 6, M_PI / 2, 
                                                              msg->angle_min, msg->angle_increment, 
                                                              msg->range_min, msg->range_max);
            std::vector<float> right_ranges = getRangesInSector(msg->ranges, -M_PI / 2, -M_PI / 6, 
                                                               msg->angle_min, msg->angle_increment, 
                                                               msg->range_min, msg->range_max);
            
            float left_clearance = getMinDistance(left_ranges);
            float right_clearance = getMinDistance(right_ranges);

            if (left_clearance > right_clearance && left_clearance > 1.0f)
            {
                linear_vel = 0.3f;
                angular_vel = 0.6f; // Reduzierte Drehgeschwindigkeit
            }
            else if (right_clearance > 1.0f)
            {
                linear_vel = 0.3f;
                angular_vel = -0.6f;
            }
            else
            {
                linear_vel = -0.2f;
                angular_vel = 0.4f;
            }
            
            // Reset für sauberen Übergang
            integral_error_ = 0.0f;
            previous_error_ = 0.0f;
            angle_history_.clear();
            break;
        }

        case RobotState::WALL_FOLLOWING:
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Zustand: WANDVERFOLGUNG - Winkel: %.2f°, Distanz: %.2fm", 
                                wall_angle * 180.0 / M_PI, wall_distance);
            
            // Winkel- und Distanzfehler kombinieren
            float angle_error = wall_angle - 0.0f; // Gewünschter Winkel ist 0
            
            // Distanzkorrektur hinzufügen
            float distance_error = wall_distance - DESIRED_WALL_DISTANCE;
            float combined_error = angle_error + 0.3f * distance_error; // Gewichtete Kombination

            angular_vel = -stabilizedPidController(combined_error, dt);
            
            // Geschwindigkeit basierend auf Lenkbewegung anpassen
            linear_vel = 0.5f; // Grundgeschwindigkeit
            if (std::abs(angular_vel) > 0.3f)
            {
                linear_vel *= (1.0f - std::abs(angular_vel) * 0.5f); // Langsamer bei starken Korrekturen
            }
            break;
        }

        case RobotState::SEEKING_OBSTACLE:
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "Zustand: HINDERNISSUCHE");

            // Finde nächstes Hindernis
            float min_range = std::numeric_limits<float>::infinity();
            int min_index = -1;

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
                float target_angle = msg->angle_min + min_index * msg->angle_increment;
                linear_vel = 0.4f;
                angular_vel = 0.6f * target_angle; // Reduzierte Drehgeschwindigkeit
            }
            else
            {
                linear_vel = 0.1f;
                angular_vel = -0.3f;
            }
            
            // Reset für sauberen Übergang
            integral_error_ = 0.0f;
            previous_error_ = 0.0f;
            angle_history_.clear();
            break;
        }
        }

        // Finale Sicherheitsbegrenzungen
        linear_vel = std::max(-0.3f, std::min(linear_vel, 0.6f));
        angular_vel = std::max(-1.0f, std::min(angular_vel, 1.0f));

        // Geschwindigkeitsbefehl senden
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
                if (range >= range_min && range <= range_max && 
                    !std::isnan(range) && !std::isinf(range))
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
    auto node = std::make_shared<StabilizedWallFollowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}