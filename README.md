# robot_ur3e_perception

- [robot\_ur3e\_perception](#robot_ur3e_perception)
  - [For searching a coffee space - REAL](#for-searching-a-coffee-space---real)
    - [Launch YoloV5](#launch-yolov5)
  - [For searching a coffee space - SIM](#for-searching-a-coffee-space---sim)
    - [Launch Camera \& TF](#launch-camera--tf)
    - [Launch Camera \& TF \& Marker](#launch-camera--tf--marker)
    - [If ran separately](#if-ran-separately)
  - [Resources](#resources)

## For searching a coffee space - REAL
### Launch YoloV5
```
cd /home/user/ros2_ws/src/robot_ur3e_perception; source venv/bin/activate
```
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 launch robot_ur3e_perception yolov5.launch.py
```

## For searching a coffee space - SIM
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

Virtual environments:
- https://medium.com/ros2-tips-and-tricks/running-ros2-nodes-in-a-python-virtual-environment-b31c1b863cdb

Parameters for camera:
- https://github.com/Marnonel6/YOLOv7_ROS2/blob/main/object_detection/launch/object_detection.launch.xml