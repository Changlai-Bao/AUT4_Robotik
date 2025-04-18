# Robotik-Praktikum

Dieses Repository enthält Code für das Robotik-Praktikum mit verschiedenen Komponenten: C++-Einführung, ROS2-Workspaces und STM32-Mikrocontroller-Projekte.

## ROS2-Projekte

```bash
# Für Versuch 1&2
ros2 run turtlesim turtlesim_node # turtle1 erstellen
rqt # in /spawn turtle2 erstellen
ros2 run turtlesim turtle_teleop_key # turtle1 per Tastatur steuern

cd Versuch_1&2/ros2_ws
colcon build
source install/setup.bash
ros2 run turtlebot_follower turtlebot_follower_node
```

```bash
# Für Versuch 3&4
cd Versuch_3&4/ros2_ws
colcon build
source install/setup.bash
ros2 run st_vl6180 st_vl6180_node
```
