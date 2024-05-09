# robot_ur3e_perception

- [robot\_ur3e\_perception](#robot_ur3e_perception)
  - [For searching a coffee space](#for-searching-a-coffee-space)
    - [Launch Camera \& Marker](#launch-camera--marker)
  - [Previous Basics](#previous-basics)
    - [Run normal image](#run-normal-image)
    - [Run depth image](#run-depth-image)
  - [Resources](#resources)

## For searching a coffee space
### Launch Camera & Marker
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 launch robot_ur3e_perception camera_and_marker.launch.py
```
Run Camera
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception camera
```
Run Marker
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception marker
```

## Previous Basics

### Run normal image
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception camera_rgb
```

### Run depth image
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception camera_mono
```

## Resources
Depth Camera Information:
- https://www.intelrealsense.com/depth-camera-d435i/