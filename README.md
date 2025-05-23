# hunavis
Utilities for HUNAVsim, NAV2, and VISualization

## Nodes
- [`people_visualizer`](hunavis/people_visualizer.py): Visualizes detected humans as markers in rviz
- [`robot_pose_publisher`](hunavis/robot_pose_publisher.py): Publishes robot pose
- [`tf_keyboard_publisher`](hunavis/tf_keyboard_publisher.py): Publishes a transform that's adjustable using keyboard

## Launch files
- [`hudet.launch.py`](launch/hudet.launch.py): Starts human detection using a Zed2 camera.
- [`map_server.launch.py`](launch/map_server.launch.py): Starts the map server without other Nav2 functionalities
- [`mars.launch.py`](launch/mars.launch.py): Starts a [HuNavSim](https://github.com/robotics-upo/hunav_sim) simulation in Gazebo
- [`tb3_custom_sim.launch.py`](launch/tb3_custom_sim.launch.py): Starts Nav2 with basic robot navigation functionality