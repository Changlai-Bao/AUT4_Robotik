#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "Vector3.h"
#include "Quaternion.h"

using namespace std::chrono_literals;

/**
 * @class XNucleoIKS01A3FusionNode
 * @brief Implementiert einen Fusionsalgorithmus für IMU-Daten zur Orientierungsschätzung
 * @date 29.04.2025
 */
class XNucleoIKS01A3FusionNode : public rclcpp::Node
{
public:
  XNucleoIKS01A3FusionNode()
      : Node("xnucleo_iks01a3_fusion_node"),
        _calibrationSamples(100),
        _cnt(0),
        _isFirstMeasurement(true),
        _alpha(0.1f) // Kann als Parameter konfiguriert werden
  {
    // Parameter für Alpha (Gewichtungsfaktor) deklarieren
    this->declare_parameter<float>("alpha", 0.1f);
    _alpha = this->get_parameter("alpha").as_double();

    // Publisher erstellen
    _pubPoseAcc = this->create_publisher<geometry_msgs::msg::PoseStamped>("poseAcc", 1);
    _pubPoseGyro = this->create_publisher<geometry_msgs::msg::PoseStamped>("poseGyro", 1);
    _pubPoseFused = this->create_publisher<geometry_msgs::msg::PoseStamped>("poseFused", 1);

    // Subscriber erstellen
    _subscription = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 1, std::bind(&XNucleoIKS01A3FusionNode::callbackIMU, this, std::placeholders::_1));

    // Gyro-Offset initialisieren
    vGyroOffset = aut4::Vector3(0.0, 0.0, 0.0);

    // Fusionierte Quaternion mit Identität initialisieren
    _qFused = aut4::Quaternion(1.0, 0.0, 0.0, 0.0);

    RCLCPP_INFO(this->get_logger(), "XNucleo IKS01A3 Fusion Node gestartet mit alpha=%.3f", _alpha);
  }

private:
  void callbackIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rclcpp::Time time = msg->header.stamp;
    static rclcpp::Time timeprev = time;
    rclcpp::Duration dT = time - timeprev;
    timeprev = time;

    // Zeile 1: Beschleunigungsvektor normieren
    aut4::Vector3 vAcc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    vAcc.normalize();

    // Zeile 2-5: Initialisierung bei ersten Messwert
    if (_isFirstMeasurement)
    {
      _vAccReference = vAcc;
      _qFused = aut4::Quaternion(1.0, 0.0, 0.0, 0.0); // Identitätsquaternion
      _isFirstMeasurement = false;
      RCLCPP_INFO(this->get_logger(), "Referenzvektor initialisiert");
    }

    // Gyroskop-Daten holen
    aut4::Vector3 vGyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // Kalibrierung des Gyroskops
    if (_cnt < _calibrationSamples)
    {
      aut4::Vector3 vGyroWeighted = vGyro;
      vGyroWeighted *= 1.0 / (double)_calibrationSamples;
      vGyroOffset += vGyroWeighted;
      _cnt++;
      return;
    }

    // Gyroskop-Offset korrigieren
    vGyro -= vGyroOffset;

    // Zeile 6-7: Berechnung des Beschleunigungsquaternions
    // Spezialfall für parallele Vektoren beachten
    double dotProduct = vAcc.dot(_vAccReference);
    aut4::Quaternion qAcc;

    if (dotProduct > 0.99999)
    {
      // Vektoren sind fast parallel
      qAcc = aut4::Quaternion(1.0, 0.0, 0.0, 0.0);
    }
    else if (dotProduct < -0.99999)
    {
      // Vektoren sind fast antiparallel - 180° Rotation um beliebige orthogonale Achse
      aut4::Vector3 axis(1.0, 0.0, 0.0);
      aut4::Vector3 crossProd = axis.cross(vAcc);
      if (crossProd.norm() < 1e-6)
      {
        // Falls erste Achse nicht geeignet, nehme andere
        axis = aut4::Vector3(0.0, 1.0, 0.0);
        crossProd = axis.cross(vAcc);
      }
      crossProd.normalize();
      qAcc = aut4::Quaternion(0.0, crossProd[0], crossProd[1], crossProd[2]);
    }
    else
    {
      // Normale Berechnung für nicht-parallele Vektoren
      qAcc = aut4::Quaternion::quaternionFromTwoVectors(vAcc, _vAccReference);
    }

    // Veröffentliche Beschleunigungsquaternion
    publishPose(_pubPoseAcc, time, 4.0, qAcc);

    // Zeile 8-9: Gyroskop-Vektor normieren und Winkel berechnen
    double gyroNorm = vGyro.norm();
    double phiW = gyroNorm * dT.seconds();

    // Zeile 10: Gyroskop-Quaternion berechnen
    aut4::Quaternion qGyro;
    if (gyroNorm > 1e-6)
    {
      vGyro *= (1.0 / gyroNorm); // Normieren für Rotationsachse
      double halfPhi = phiW / 2.0;
      qGyro = aut4::Quaternion(cos(halfPhi),
                               vGyro[0] * sin(halfPhi),
                               vGyro[1] * sin(halfPhi),
                               vGyro[2] * sin(halfPhi));
    }
    else
    {
      // Bei sehr kleinen Rotationen: Identitätsquaternion
      qGyro = aut4::Quaternion(1.0, 0.0, 0.0, 0.0);
    }

    // Veröffentliche Gyroskop-Quaternion
    static aut4::Quaternion qGyroGesamt(1.0, 0.0, 0.0, 0.0);
    qGyroGesamt *= qGyro;
    qGyroGesamt.normalize();
    publishPose(_pubPoseGyro, time, 2.0, qGyroGesamt);

    // Zeile 11: Aktualisierung des fusionierten Quaternions mit Gyro-Daten
    _qFused *= qGyro;
    _qFused.normalize();

    // Zeile 12: Berechne driftbehaftete Referenz
    aut4::Vector3 vGyroRef = vAcc; // Aktueller Beschleunigungsvektor
    _qFused.rotate(vGyroRef);      // Rotation auf Referenzvektor anwenden
    vGyroRef.normalize();

    // Zeile 13-15: Drift-Vektor und -Winkel berechnen
    aut4::Vector3 vDrift;
    double phiDrift;

    // Spezialfall: Parallele Vektoren
    dotProduct = vGyroRef.dot(_vAccReference);
    if (dotProduct > 0.99999)
    {
      // Keine Drift-Korrektur notwendig
      phiDrift = 0.0;
    }
    else if (dotProduct < -0.99999)
    {
      // 180° Rotation
      phiDrift = M_PI;
      // Beliebige orthogonale Achse wählen
      aut4::Vector3 axis(1.0, 0.0, 0.0);
      vDrift = axis.cross(_vAccReference);
      if (vDrift.norm() < 1e-6)
      {
        axis = aut4::Vector3(0.0, 1.0, 0.0);
        vDrift = axis.cross(_vAccReference);
      }
    }
    else
    {
      // Normale Berechnung der Drift
      vDrift = vGyroRef.cross(_vAccReference);
      phiDrift = acos(dotProduct);
    }
    vDrift.normalize();

    // Zeile 16: Drift-Kompensationsquaternion berechnen
    double halfAlphaPhi = _alpha * phiDrift / 2.0;
    aut4::Quaternion qDrift(cos(halfAlphaPhi),
                            vDrift[0] * sin(halfAlphaPhi),
                            vDrift[1] * sin(halfAlphaPhi),
                            vDrift[2] * sin(halfAlphaPhi));

    // Zeile 17: Fusioniertes Quaternion aktualisieren
    _qFused = qDrift * _qFused;

    // Veröffentliche fusioniertes Quaternion
    publishPose(_pubPoseFused, time, 0.0, _qFused);
  }

  // Hilfsfunktion zum Veröffentlichen einer Pose
  void publishPose(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher,
                   rclcpp::Time time, double x_pos, const aut4::Quaternion &quat)
  {
    geometry_msgs::msg::PoseStamped msgPose;
    msgPose.header.frame_id = "map";
    msgPose.header.stamp = time;
    msgPose.pose.position.x = x_pos;
    msgPose.pose.position.y = 0.0;
    msgPose.pose.position.z = 0.0;
    msgPose.pose.orientation.x = quat.x;
    msgPose.pose.orientation.y = quat.y;
    msgPose.pose.orientation.z = quat.z;
    msgPose.pose.orientation.w = quat.w;
    publisher->publish(msgPose);
  }

  // Subscriber und Publisher
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _subscription;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pubPoseAcc;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pubPoseGyro;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pubPoseFused;

  // Kalibrierungsvariablen
  int _calibrationSamples;
  int _cnt;
  aut4::Vector3 vGyroOffset;

  // Zustands- und Filtervariablen
  bool _isFirstMeasurement;
  aut4::Vector3 _vAccReference;
  aut4::Quaternion _qFused;
  float _alpha; // Gewichtungsfaktor für die Fusion
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XNucleoIKS01A3FusionNode>());
  rclcpp::shutdown();
  return 0;
}