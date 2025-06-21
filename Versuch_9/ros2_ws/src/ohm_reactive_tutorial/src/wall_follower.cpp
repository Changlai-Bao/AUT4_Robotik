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
            "/robot1/laser",                          // topic name
            10,                                       // queue size
            std::bind(&WallFollowNode::lidarCallback, // callback function
                      this,
                      std::placeholders::_1));

        // Create a publisher for the cmd_vel topic
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/robot1/cmd_vel",
            10);

        // Initialisierung der Wall-Following-Parameter
        sollabstand_zur_wand_ = 1.5;       // Gewünschter Abstand zur Wand in Metern
        max_linear_geschwindigkeit_ = 0.4; // Maximale Vorwärtsgeschwindigkeit
        min_linear_geschwindigkeit_ = 0.1; // Minimale Vorwärtsgeschwindigkeit

        // Einfache P-Regler Parameter (ohne Integral für Stabilität)
        kp_ = 0.8;              // Reduzierte Proportionalverstärkung
        ki_ = 0.0;              // Kein Integral-Teil
        integral_fehler_ = 0.0; // Akkumulierter Integralfehler

        // Winkelindizes für Wanderkennung (-180° bis +180°, 10° Schritte, 36 Strahlen)
        // Index 0: -180°, Index 9: -90° (rechts), Index 18: 0° (vorne), Index 27: 90° (links)
        rechter_wand_index_ = 9;   // -90° für rechte Seite
        vorderer_wand_index_ = 18; // 0° für Hinderniserkennung voraus
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
        // Überprüfung, ob ausreichend Scan-Daten vorhanden sind
        if (msg->ranges.size() < 36)
        {
            RCLCPP_WARN(this->get_logger(), "Nicht genügend Laser-Scan-Daten erhalten");
            return;
        }

        // Extrahiere relevante Distanzen aus den Laser-Scan-Daten
        float distanz_rechte_wand = msg->ranges[rechter_wand_index_]; // -90° (rechte Seite)
        float distanz_vorne = msg->ranges[vorderer_wand_index_];      // 0° (vorne)
        float distanz_rechts_vorne = msg->ranges[14];                 // -40° (rechts vorne)

        // Werte überprüfen und auf max_range setzen, wenn sie ungültig sind
        if (std::isinf(distanz_rechte_wand) || std::isnan(distanz_rechte_wand))
        {
            distanz_rechte_wand = msg->range_max;
        }
        if (std::isinf(distanz_vorne) || std::isnan(distanz_vorne))
        {
            distanz_vorne = msg->range_max;
        }
        if (std::isinf(distanz_rechts_vorne) || std::isnan(distanz_rechts_vorne))
        {
            distanz_rechts_vorne = msg->range_max;
        }

        // Berechnung der Wandverfolgungsgeschwindigkeit
        float linear_geschwindigkeit = max_linear_geschwindigkeit_;
        float winkel_korrektur = 0.0;

        // Regel 1: Hindernis voraus - links abbiegen (höchste Priorität)
        if (distanz_vorne < 2.5 || distanz_rechts_vorne < 2.0)
        {
            winkel_korrektur = 0.5;
            linear_geschwindigkeit = 0.2;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Hindernis vorne - links abbiegen");
        }
        // Regel 2: Rechte Seite zu nah - leichte Linksdrehung
        else if (distanz_rechte_wand < 1.0)
        {
            winkel_korrektur = 0.2;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Zu nah an rechter Wand");
        }
        // Regel 3: Zu weit rechts - leichte Rechtskurve
        else if (distanz_rechte_wand > 2.5)
        {
            winkel_korrektur = -0.2;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Zu weit von rechter Wand");
        }
        // Regel 4: Richtiger Abstand - geradeaus
        else
        {
            winkel_korrektur = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Geradeaus fahren - guter Abstand");
        }

        // Output der Wandverfolgungsinformationen
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "rechts:%.1fm vorne:%.1fm rechts vorne:%.1fm → Winkel:%.2f",
                             distanz_rechte_wand, distanz_vorne, distanz_rechts_vorne, winkel_korrektur);

        // Create a Twist message with the desired velocity
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_geschwindigkeit; // Vorwärtsgeschwindigkeit
        cmd_vel_msg.angular.z = winkel_korrektur;      // Winkelgeschwindigkeit für Wandverfolgung

        // Publish the cmd_vel message
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Wall-Following Parameter
    float sollabstand_zur_wand_;       // Gewünschter Abstand zur Wand
    float max_linear_geschwindigkeit_; // Maximale Vorwärtsgeschwindigkeit
    float min_linear_geschwindigkeit_; // Minimale Vorwärtsgeschwindigkeit

    // PI-Regler Parameter
    float kp_;              // Proportionalverstärkung
    float ki_;              // Integralverstärkung
    float integral_fehler_; // Akkumulierter Integralfehler

    // Laser-Scanner Indizes
    int rechter_wand_index_;  // Index für rechte Wanderkennung
    int vorderer_wand_index_; // Index für Hinderniserkennung voraus
};

/**
 * @brief
 *
 * @param argc  number of arguments provided from the command line
 * @param argv  array of arguments provided from the command line
 * @return int  0
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFollowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}