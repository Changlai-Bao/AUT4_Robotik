/**
 * @file stable_wall_follower.cpp      
 * @author Prof. Dr. Christian Pfitzner (christian.pfitzner@th-nuernberg.de)
 * @brief Stabiler Wall-Following Algorithmus mit verbesserter Kontrolle
 * @version 0.3
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

/**
 * @class StableWallFollowNode
 * @brief Stabiler Wall-Following mit reduzierter Zustandskomplexität
 */
class StableWallFollowNode : public rclcpp::Node
{
public:
    StableWallFollowNode() : Node("stable_wall_follow_node")
    {
        // Subscriber für Lidar-Daten
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot1/laser", 10,
            std::bind(&StableWallFollowNode::lidarCallback, this, std::placeholders::_1));

        // Publisher für Geschwindigkeitsbefehle
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        // Initialisierung
        integral_error_ = 0.0f;
        previous_time_ = this->now();
        follow_right_wall_ = true;  // Standardmäßig rechte Wand folgen
        state_change_timer_ = 0.0f;
        
        RCLCPP_INFO(this->get_logger(), "Stabiler Wall-Follower gestartet");
    }

private:
    /**
     * @brief Berechnet den Winkel zur Wand (vereinfacht und stabiler)
     */
    float calculateWallAngle(const std::vector<float> &ranges,
                             float angle_min, float angle_increment,
                             float range_min, float range_max)
    {
        // Fixe Punkte für rechte Wand (wie im Original)
        float point1_angle = -M_PI / 2;       // -90°
        float point2_angle = -M_PI / 2 + 0.5; // -60°

        int idx1 = static_cast<int>((point1_angle - angle_min) / angle_increment);
        int idx2 = static_cast<int>((point2_angle - angle_min) / angle_increment);

        if (idx1 < 0 || idx1 >= static_cast<int>(ranges.size()) ||
            idx2 < 0 || idx2 >= static_cast<int>(ranges.size())) {
            return std::numeric_limits<float>::quiet_NaN();
        }

        float range1 = ranges[idx1];
        float range2 = ranges[idx2];

        if (range1 < range_min || range1 > range_max || std::isnan(range1) || std::isinf(range1) ||
            range2 < range_min || range2 > range_max || std::isnan(range2) || std::isinf(range2)) {
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
        if (magnitude_b < 1e-6) {
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

        if (idx < 0 || idx >= static_cast<int>(ranges.size())) {
            return std::numeric_limits<float>::infinity();
        }

        float range = ranges[idx];
        if (range < range_min || range > range_max || std::isnan(range) || std::isinf(range)) {
            return std::numeric_limits<float>::infinity();
        }

        return range;
    }

    /**
     * @brief Verbesserte PI-Regelung mit stabileren Parametern
     */
    float stablePiController(float angle_error, float dt)
    {
        // Konservativere Parameter für Stabilität
        float kp = 1.5f;  // Reduziert von 2.0f
        float ki = 0.3f;  // Reduziert von 0.5f

        // Integralterm mit Anti-Windup
        integral_error_ += angle_error * dt;
        float max_integral = 1.0f;
        integral_error_ = std::max(-max_integral, std::min(max_integral, integral_error_));

        // PI-Ausgabe
        float control_output = kp * angle_error + ki * integral_error_;
        
        // Begrenze Ausgabe für Stabilität
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
            ranges, -M_PI/6, M_PI/6,  // ±30° Frontbereich
            angle_min, angle_increment, range_min, range_max);
        
        float min_front_distance = getMinDistance(front_ranges);
        
        // Hysterese zur Vermeidung von Flackern
        static bool obstacle_detected = false;
        float threshold_detect = 1.8f;   // Schwelle für Erkennung
        float threshold_clear = 2.2f;    // Schwelle für Freigabe
        
        if (!obstacle_detected && min_front_distance < threshold_detect) {
            obstacle_detected = true;
        } else if (obstacle_detected && min_front_distance > threshold_clear) {
            obstacle_detected = false;
        }
        
        return obstacle_detected;
    }

    /**
     * @brief Hauptcallback mit vereinfachter Logik
     */
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Zeitberechnung
        rclcpp::Time current_time = this->now();
        float dt = (current_time - previous_time_).seconds();
        if (dt <= 0.0f || dt > 0.1f) dt = 0.05f;
        previous_time_ = current_time;

        // Inkrementiere State-Change Timer
        state_change_timer_ += dt;

        // Sensordaten analysieren
        float wall_angle = calculateWallAngle(msg->ranges, msg->angle_min, msg->angle_increment,
                                            msg->range_min, msg->range_max);
        float wall_distance = calculateWallDistance(msg->ranges, msg->angle_min, msg->angle_increment,
                                                  msg->range_min, msg->range_max);
        bool front_obstacle = detectFrontObstacle(msg->ranges, msg->angle_min, msg->angle_increment,
                                                 msg->range_min, msg->range_max);

        // Standardwerte
        float linear_vel = 0.0f;
        float angular_vel = 0.0f;

        if (front_obstacle) {
            // **HINDERNISVERMEIDUNG** - Einfache aber effektive Strategie
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Hindernis erkannt - weiche aus");
            
            // Prüfe verfügbaren Platz links und rechts
            std::vector<float> left_ranges = getRangesInSector(
                msg->ranges, M_PI/6, M_PI/2, msg->angle_min, msg->angle_increment, 
                msg->range_min, msg->range_max);
            std::vector<float> right_ranges = getRangesInSector(
                msg->ranges, -M_PI/2, -M_PI/6, msg->angle_min, msg->angle_increment,
                msg->range_min, msg->range_max);

            float left_clearance = getMinDistance(left_ranges);
            float right_clearance = getMinDistance(right_ranges);

            // Wähle Richtung mit mehr Platz
            if (left_clearance > right_clearance && left_clearance > 1.0f) {
                // Links ausweichen
                linear_vel = 0.3f;
                angular_vel = 0.8f;  // Links drehen
            } else if (right_clearance > 1.0f) {
                // Rechts ausweichen
                linear_vel = 0.3f;
                angular_vel = -0.8f; // Rechts drehen
            } else {
                // Notfall: Rückwärts
                linear_vel = -0.2f;
                angular_vel = 0.5f;
            }

            // Reset Integralterm bei Hindernisvermeidung
            integral_error_ = 0.0f;

        } else if (!std::isnan(wall_angle) && !std::isinf(wall_distance) && wall_distance < 4.0f) {
            // **NORMALE WANDVERFOLGUNG**
            float desired_wall_distance = 1.3f;
            float desired_wall_angle = 0.0f;

            float angle_error = wall_angle - desired_wall_angle;
            float distance_factor = 1.0f;

            // Distanzkorrektur nur bei größeren Abweichungen
            if (wall_distance < desired_wall_distance - 0.3f) {
                distance_factor = 1.2f;  // Leicht weg von der Wand
            } else if (wall_distance > desired_wall_distance + 0.3f) {
                distance_factor = 0.8f;  // Leicht zur Wand hin
            }

            angular_vel = -stablePiController(angle_error, dt) * distance_factor;
            linear_vel = 0.5f;

            // Reduziere Geschwindigkeit bei starken Korrekturen
            if (std::abs(angular_vel) > 0.4f) {
                linear_vel *= 0.7f;
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Wandverfolgung - Winkel: %.2f°, Distanz: %.2fm, ω: %.2f",
                wall_angle * 180.0 / M_PI, wall_distance, angular_vel);

        } else {
            // **WANDSUCHE** - Langsam vorwärts und nach rechts schauen
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Suche Wand - Distanz: %.2fm", wall_distance);
            
            linear_vel = 0.3f;
            angular_vel = -0.3f;  // Leicht nach rechts drehen
            
            // Reset Integralterm bei Wandsuche
            integral_error_ = 0.0f;
        }

        // Sicherheitsbegrenzungen
        linear_vel = std::max(-0.3f, std::min(linear_vel, 0.6f));
        angular_vel = std::max(-1.5f, std::min(angular_vel, 1.5f));

        // Bewegungsbefehl senden
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_vel;
        cmd_vel_msg.angular.z = angular_vel;
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }

    /**
     * @brief Hilfsfunktionen (unverändert)
     */
    std::vector<float> getRangesInSector(const std::vector<float> &ranges,
                                         float angle_min, float angle_max,
                                         float scan_angle_min, float scan_angle_increment,
                                         float range_min, float range_max)
    {
        std::vector<float> sector_ranges;

        for (size_t i = 0; i < ranges.size(); ++i) {
            float current_angle = scan_angle_min + i * scan_angle_increment;

            if (current_angle >= angle_min && current_angle <= angle_max) {
                float range = ranges[i];
                if (range >= range_min && range <= range_max &&
                    !std::isnan(range) && !std::isinf(range)) {
                    sector_ranges.push_back(range);
                }
            }
        }

        return sector_ranges;
    }

    float getMinDistance(const std::vector<float> &ranges)
    {
        if (ranges.empty()) {
            return std::numeric_limits<float>::infinity();
        }
        return *std::min_element(ranges.begin(), ranges.end());
    }

    // Member-Variablen
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    
    float integral_error_;
    rclcpp::Time previous_time_;
    bool follow_right_wall_;
    float state_change_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StableWallFollowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}