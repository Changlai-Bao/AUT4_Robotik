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
        AVOIDING_OBSTACLE,   // Hindernisausweichung
        CORNER_TURNING       // Eckendrehung (neuer Zustand)
    };

    // ROS2 Subscriber und Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Regelungsvariablen
    float integral_error_;
    float previous_error_;           // Für D-Anteil (falls benötigt)
    rclcpp::Time previous_time_;
    RobotState current_state_;
    RobotState previous_state_;      // Vorheriger Zustand für bessere Übergänge

    // Eckenbehandlung
    rclcpp::Time corner_start_time_; // Zeitpunkt des Eckenbeginns
    float last_wall_distance_;       // Letzte bekannte Wanddistanz
    bool wall_lost_;                 // Flag für verlorene Wand

    // Glättungsfilter für Winkelberechnung
    std::deque<float> angle_history_;
    static const size_t ANGLE_FILTER_SIZE = 5;

    // Konfigurierbare Parameter für Geschwindigkeitsoptimierung
    const float DESIRED_WALL_DISTANCE = 1.3f;     // Gewünschter Wandabstand
    const float DEADBAND_THRESHOLD = 0.05f;       // Totband für kleine Korrekturen
    const float MAX_INTEGRAL = 0.8f;              // Begrenzung des I-Anteils
    
    // Verbesserte PI-Parameter für stabilere Regelung
    const float KP = 1.2f;                        // Reduzierter P-Anteil
    const float KI = 0.2f;                        // Reduzierter I-Anteil
    const float KD = 0.1f;                        // Kleiner D-Anteil für Dämpfung

    // Geschwindigkeitsparameter für schnellere Navigation
    const float MAX_LINEAR_SPEED = 1.0f;          // Maximale Lineargeschwindigkeit
    const float HIGH_SPEED = 0.8f;                // Hohe Geschwindigkeit bei geringer Krümmung
    const float MEDIUM_SPEED = 0.6f;              // Mittlere Geschwindigkeit bei normaler Navigation
    const float LOW_SPEED = 0.3f;                 // Niedrige Geschwindigkeit bei scharfen Kurven
    const float EMERGENCY_SPEED = 0.1f;           // Notfallgeschwindigkeit
    
    // Adaptive Geschwindigkeitssteuerung
    const float SPEED_ADAPTATION_FACTOR = 0.7f;   // Faktor für Geschwindigkeitsanpassung
    const float CORNER_DETECTION_THRESHOLD = 0.3f; // Schwellwert für Eckenerkennung
    
    // Eckenbehandlungsparameter
    const float WALL_LOST_THRESHOLD = 5.0f;       // Schwellwert für verlorene Wand
    const float CORNER_TURN_DURATION = 2.0f;      // Maximale Eckendrehzeit in Sekunden
    const float CORNER_ANGULAR_VELOCITY = 0.8f;   // Drehgeschwindigkeit bei Ecken

public:
    StabilizedWallFollowNode() : Node("stabilized_wall_follow_node")
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
        previous_state_ = RobotState::SEEKING_OBSTACLE;
        corner_start_time_ = this->now();
        last_wall_distance_ = std::numeric_limits<float>::infinity();
        wall_lost_ = false;

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
     * @brief Intelligente Geschwindigkeitsberechnung basierend auf Navigationszustand
     */
    float calculateAdaptiveSpeed(float angular_velocity, float wall_distance, 
                                const std::vector<float> &ranges,
                                float angle_min, float angle_increment)
    {
        // 1. Grundgeschwindigkeit basierend auf Lenkbewegung
        float base_speed = HIGH_SPEED;
        float angular_factor = std::abs(angular_velocity);
        
        if (angular_factor > 0.5f)
        {
            base_speed = LOW_SPEED;  // Scharfe Kurve
        }
        else if (angular_factor > 0.2f)
        {
            base_speed = MEDIUM_SPEED;  // Normale Kurve
        }
        
        // 2. Eckenerkennung für proaktive Verlangsamung
        bool corner_detected = detectCorner(ranges, angle_min, angle_increment);
        if (corner_detected)
        {
            base_speed = std::min(base_speed, MEDIUM_SPEED);
        }
        
        // 3. Freier Raum vor dem Roboter
        float front_clearance = getFrontClearance(ranges, angle_min, angle_increment);
        if (front_clearance > 3.0f)
        {
            base_speed = std::min(MAX_LINEAR_SPEED, base_speed * 1.2f); // Boost bei freier Strecke
        }
        else if (front_clearance < 1.5f)
        {
            base_speed *= 0.7f; // Verlangsamung bei Hindernissen
        }
        
        // 4. Wandabstand berücksichtigen
        if (wall_distance < DESIRED_WALL_DISTANCE - 0.5f)
        {
            base_speed *= 0.8f; // Vorsichtiger bei zu geringem Wandabstand
        }
        
        return std::max(EMERGENCY_SPEED, std::min(MAX_LINEAR_SPEED, base_speed));
    }

    /**
     * @brief Verbesserte Eckenerkennung basierend auf Wandverlust
     */
    bool detectWallLoss(float wall_distance)
    {
        // Wand ist verloren, wenn die Distanz plötzlich sehr groß wird
        if (wall_distance > WALL_LOST_THRESHOLD && last_wall_distance_ < DESIRED_WALL_DISTANCE * 2.0f)
        {
            return true;
        }
        return false;
    }

    /**
     * @brief Erkennt das Ende einer Ecke (neue Wand gefunden)
     */
    bool detectNewWall(float wall_distance, float wall_angle)
    {
        // Neue Wand erkannt, wenn:
        // 1. Distanz wieder in normalem Bereich
        // 2. Winkel ist akzeptabel
        return (wall_distance < DESIRED_WALL_DISTANCE * 2.0f && 
                !std::isnan(wall_angle) && 
                std::abs(wall_angle) < M_PI/3);
    }
    bool detectCorner(const std::vector<float> &ranges, float angle_min, float angle_increment)
    {
        // Analysiere rechte Seite für Ecken/Diskontinuitäten
        std::vector<float> right_ranges = getRangesInSector(ranges, -M_PI/2 - 0.3f, -M_PI/2 + 0.3f,
                                                            angle_min, angle_increment, 0.1f, 8.0f);
        
        if (right_ranges.size() < 3) return false;
        
        // Suche nach großen Sprüngen in der Entfernung (Ecken)
        for (size_t i = 1; i < right_ranges.size() - 1; ++i)
        {
            float diff1 = std::abs(right_ranges[i] - right_ranges[i-1]);
            float diff2 = std::abs(right_ranges[i+1] - right_ranges[i]);
            
            if (diff1 > 0.5f || diff2 > 0.5f) // Diskontinuität erkannt
            {
                return true;
            }
        }
        
        return false;
    }

    /**
     * @brief Berechnet freien Raum vor dem Roboter
     */
    float getFrontClearance(const std::vector<float> &ranges, float angle_min, float angle_increment)
    {
        std::vector<float> front_ranges = getRangesInSector(ranges, -M_PI/8, M_PI/8,
                                                           angle_min, angle_increment, 0.1f, 8.0f);
        
        return getMinDistance(front_ranges);
    }
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

        // Erweiterte Zustandslogik mit Eckenbehandlung
        previous_state_ = current_state_;
        
        // Zustandsübergänge mit verbesserter Eckenlogik
        if (front_obstacle)
        {
            current_state_ = RobotState::AVOIDING_OBSTACLE;
            wall_lost_ = false;
        }
        else if (current_state_ == RobotState::CORNER_TURNING)
        {
            // Prüfe ob Eckendrehung beendet werden kann
            rclcpp::Time current_time = this->now();
            double corner_duration = (current_time - corner_start_time_).seconds();
            
            if (detectNewWall(wall_distance, wall_angle))
            {
                // Neue Wand gefunden - zurück zur normalen Wandverfolgung
                current_state_ = RobotState::WALL_FOLLOWING;
                wall_lost_ = false;
                RCLCPP_INFO(this->get_logger(), "Neue Wand gefunden - Ecke beendet");
            }
            else if (corner_duration > CORNER_TURN_DURATION)
            {
                // Timeout - zurück zur Hindernissuche
                current_state_ = RobotState::SEEKING_OBSTACLE;
                wall_lost_ = false;
                RCLCPP_WARN(this->get_logger(), "Ecken-Timeout - zurück zur Suche");
            }
            // Sonst bleibe im Eckenmodus
        }
        else if (!std::isnan(wall_angle) && wall_distance < 4.0f)
        {
            // Normale Wandverfolgung möglich
            if (detectWallLoss(wall_distance) && previous_state_ == RobotState::WALL_FOLLOWING)
            {
                // Wand verloren - beginne Eckendrehung
                current_state_ = RobotState::CORNER_TURNING;
                corner_start_time_ = this->now();
                wall_lost_ = true;
                RCLCPP_INFO(this->get_logger(), "Wand verloren - beginne Eckendrehung");
            }
            else
            {
                current_state_ = RobotState::WALL_FOLLOWING;
                wall_lost_ = false;
            }
        }
        else
        {
            // Keine Wand erkennbar
            if (previous_state_ == RobotState::WALL_FOLLOWING)
            {
                // Gerade Wandverfolgung verloren - beginne Eckendrehung
                current_state_ = RobotState::CORNER_TURNING;
                corner_start_time_ = this->now();
                wall_lost_ = true;
                RCLCPP_INFO(this->get_logger(), "Wandverfolgung verloren - beginne Eckendrehung");
            }
            else
            {
                current_state_ = RobotState::SEEKING_OBSTACLE;
                wall_lost_ = false;
            }
        }

        // Aktualisiere letzte bekannte Wanddistanz
        if (wall_distance < WALL_LOST_THRESHOLD)
        {
            last_wall_distance_ = wall_distance;
        }

        float linear_vel = 0.0f;
        float angular_vel = 0.0f;

        switch (current_state_)
        {
        case RobotState::CORNER_TURNING:
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "Zustand: ECKENDREHUNG - Suche neue Wand");
            
            // Kontinuierliche Rechtsdrehung bei moderater Geschwindigkeit
            linear_vel = LOW_SPEED;                    // Langsam vorwärts
            angular_vel = -CORNER_ANGULAR_VELOCITY;    // Konstante Rechtsdrehung
            
            // Überprüfe ob es sicher ist, etwas schneller zu drehen
            std::vector<float> right_ranges = getRangesInSector(msg->ranges, -M_PI/2, -M_PI/6,
                                                               msg->angle_min, msg->angle_increment,
                                                               msg->range_min, msg->range_max);
            float right_clearance = getMinDistance(right_ranges);
            
            if (right_clearance > 2.0f)
            {
                // Mehr Freiraum rechts - schneller drehen
                angular_vel = -1.2f;
            }
            
            // Reset für sauberen Übergang
            integral_error_ = 0.0f;
            previous_error_ = 0.0f;
            angle_history_.clear();
            break;
        }

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

            // Aggressivere Ausweichmanöver für schnellere Navigation
            if (left_clearance > right_clearance && left_clearance > 1.0f)
            {
                linear_vel = MEDIUM_SPEED;      // Erhöhte Geschwindigkeit
                angular_vel = 1.0f;             // Schnellere Drehung
            }
            else if (right_clearance > 1.0f)
            {
                linear_vel = MEDIUM_SPEED;
                angular_vel = -1.0f;
            }
            else
            {
                linear_vel = -LOW_SPEED;        // Schnelleres Rückwärtsfahren
                angular_vel = 0.8f;
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
            
            // Intelligente Geschwindigkeitssteuerung
            linear_vel = calculateAdaptiveSpeed(angular_vel, wall_distance, msg->ranges,
                                              msg->angle_min, msg->angle_increment);
            
            // Zusätzliche Geschwindigkeitsbegrenzung nur bei extremen Lenkbewegungen
            if (std::abs(angular_vel) > 0.8f)
            {
                linear_vel *= 0.6f; // Nur bei sehr scharfen Kurven stark verlangsamen
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
                
                // Schnellere Hindernisnäherung
                linear_vel = HIGH_SPEED;           // Erhöhte Suchgeschwindigkeit
                angular_vel = 1.2f * target_angle; // Schnellere Orientierung
                
                // Begrenzen bei sehr scharfen Winkeln
                if (std::abs(target_angle) > M_PI/3)
                {
                    linear_vel = MEDIUM_SPEED;
                }
            }
            else
            {
                linear_vel = MEDIUM_SPEED;         // Schnellere Standardsuche
                angular_vel = -0.6f;               // Schnellere Drehung
            }
            
            // Reset für sauberen Übergang
            integral_error_ = 0.0f;
            previous_error_ = 0.0f;
            angle_history_.clear();
            break;
        }
        }

        // Finale Sicherheitsbegrenzungen mit höheren Maximalwerten
        linear_vel = std::max(-0.5f, std::min(linear_vel, MAX_LINEAR_SPEED));
        angular_vel = std::max(-1.5f, std::min(angular_vel, 1.5f));

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