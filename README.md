# robot_ur3e_perception

- [robot\_ur3e\_perception](#robot_ur3e_perception)
  - [For searching a coffee space](#for-searching-a-coffee-space)
    - [Launch Camera \& TF](#launch-camera--tf)
    - [Launch Camera \& TF \& Marker](#launch-camera--tf--marker)
    - [If ran separately](#if-ran-separately)
  - [Resources](#resources)

## For searching a coffee space
### Launch Camera & TF
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 launch robot_ur3e_perception camera_tf.launch.py
```
### Launch Camera & TF & Marker
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 launch robot_ur3e_perception camera_tf_marker.launch.py
```
### If ran separately
Run Camera
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception camera
```
Run Transform
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception transform
```
Run Marker
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception marker
```

## Resources
Depth Camera Information:
- https://www.intelrealsense.com/depth-camera-d435i/
- 
