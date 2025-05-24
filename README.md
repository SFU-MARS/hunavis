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

## Suggested real-world setup for static Zed2 camera(s)
(TODO: Setups involving multiple cameras, which would mostly require repeating the steps below, each time specifying a different camera name)
1. Set up Zed2 camera(s) in the environment, with camera launch arguments defined in `<zed_launch_args_file>` ([example](params/zed_launch_args.yaml)) along with zed node and tf publisher parameters defined in `<zed_and_tf_params_file>` ([example](params/zed_common.yaml)).
2. Create a map of the environment and save it to `<map_path>`
3. Launch map server, and optionally note the approximate positions of Zed2 cameras
    ```bash
    ros2 launch hunavis map_server.launch.py use_simulator:=False map_path:=<map_path>
    ```
    - Tip: Use the "Publish Point" feature in `rviz`, which publishes mouse clicks to `/clicked_point`. tf publisher parameters can be updated based on this.
4. Launch human detection
    ```bash
    ros2 launch hunavis hudet.launch.py use_simulator:=False zed_launch_args_file:=<zed_launch_args_file>
    ```
    - If this is the first time deep learning models are run on the camera, the Zed SDK will begin to optimize them. Optionally, follow instructions [here]() to optimize the models manually. For example, the following optimizes all the models that come with the camera:
        ```bash
        ZED_Diagnostic -ais 0
        ```
5. Run tf publisher node to adjust camera pose with respect to the map.
    ```bash
    ros2 run hunavis tf_keyboard_publisher --ros-args --params-file <zed_and_tf_params_file>
    ```
    - Optionally, `<zed_and_tf_params_file>` can be updated with the fine-tuned tf 
