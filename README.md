# Robotik-Praktikum

Dieses Repository enthält Code für das Robotik-Praktikum mit verschiedenen Komponenten: C++ Einführung, ROS2-Workspaces und STM32-Mikrocontroller-Projekte.

## ROS2-Projekte

```bash
# Für Versuch 1&2
# turtle1 erstellen
ros2 run turtlesim turtlesim_node

# in /spawn turtle2 erstellen
rqt

# turtle1 per Tastatur steuern
ros2 run turtlesim turtle_teleop_key

cd AUT4_Robotik/Versuch_1&2/ros2_ws
colcon build
source install/setup.bash
ros2 run turtlebot_follower turtlebot_follower_node
```

```bash
# Für Versuch 3&4
# Sensor testen durch Terminalfenster
minicom -D /dev/ttyACM0

cd AUT4_Robotik/Versuch_3&4/ros2_ws
colcon build
source install/setup.bash
ros2 run st_vl6180 st_vl6180_node
```

```bash
# Für Versuch 5 Teil 1
cd AUT4_Robotik/Versuch_5/ros2_ws
colcon build
source install/setup.bash

# Temperatur im Terminalfenster ausgeben
ros2 run xnucleo_iks01a3 xnucleo_iks01a3_node
# In neuem Terminalfenster öffnen, Sensordaten ausgeben
ros2 topic echo /imu/data_raw
```

```bash
# Für Versuch 5 Teil 2
cd AUT4_Robotik/Versuch_5/ros2_ws
colcon build
source install/setup.bash

# Temperatur im Terminalfenster ausgeben
ros2 run xnucleo_iks01a3 xnucleo_iks01a3_node

# In neuem Terminalfenster öffnen, FusionNode starten
source install/setup.bash
ros2 run xnucleo_iks01a3 xnucleo_iks01a3_fusion_node

# 3D-Visualisierung, topic hinzufügen
rviz2

# Falls Daten in neuem Terminalfenster ausgeben möchten
ros2 topic echo /poseFused
ros2 topic echo /poseAcc
ros2 topic echo /poseGyro
```

```bash
# Für Versuch 9
cd AUT4_Robotik/Versuch_9/ros2_ws
colcon build
source install/setup.bash

# ohm_mecanum_sim starten
ros2 run ohm_mecanum_sim ohm_mecanum_sim_node

# In neuem Terminalfenster öffnen, wall_follower starten
source install/setup.bash
ros2 run ohm_reactive_tutorial wall_follower
```
