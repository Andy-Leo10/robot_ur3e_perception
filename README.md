# robot_ur3e_perception

```
git clone https://github.com/Andy-Leo10/robot_ur3e_perception.git
```

- [robot\_ur3e\_perception](#robot_ur3e_perception)
  - [Install libraries](#install-libraries)
  - [For searching a coffee space - SIM](#for-searching-a-coffee-space---sim)
    - [Launch YoloV5 \& TF](#launch-yolov5--tf)
    - [Launch Camera \& TF](#launch-camera--tf)
    - [Launch Camera \& TF \& Marker](#launch-camera--tf--marker)
    - [If ran separately](#if-ran-separately)
  - [For searching a coffee space - REAL](#for-searching-a-coffee-space---real)
    - [Launch YoloV5 \& TF](#launch-yolov5--tf-1)
  - [Resources](#resources)

---

<details>
<summary><b>Setup and download libraries</b></summary>

## Install libraries
move to pkg path and use:
```
python3 -m venv venv
cd /home/user/ros2_ws/src/coffee-dispenser-project/robot_ur3e_perception; source venv/bin/activate
pip install -r requirements.txt
```

</details>

---

<details>
<summary><b>SIMULATED ROBOT</b></summary>

## For searching a coffee space - SIM
### Launch YoloV5 & TF
```
export PYTHONPATH=$PYTHONPATH:/home/user/ros2_ws/src/coffee-dispenser-project/robot_ur3e_perception/venv/lib/python3.10/site-packages/
```
```
cd /home/user/ros2_ws/src/coffee-dispenser-project/robot_ur3e_perception; source venv/bin/activate
```
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception --symlink-install; source install/setup.bash; ros2 launch robot_ur3e_perception alt_yolov5_tf.launch.py
```
### Launch Camera & TF
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 launch robot_ur3e_perception sim_dip_tf.launch.py
```
### Launch Camera & TF & Marker
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 launch robot_ur3e_perception sim_dip_tf_marker.launch.py
```
### If ran separately
Run Camera
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception sim_dip
```
Run Transform
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception sim_transform
```
Run Marker
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception sim_marker
```

</details>

---

<details>
<summary><b>REAL ROBOT</b></summary>

## For searching a coffee space - REAL
### Launch YoloV5 & TF
```
export PYTHONPATH=$PYTHONPATH:/home/user/ros2_ws/src/coffee-dispenser-project/robot_ur3e_perception/venv/lib/python3.10/site-packages/
```
```
cd /home/user/ros2_ws/src/coffee-dispenser-project/robot_ur3e_perception; source venv/bin/activate
```
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception --symlink-install; source install/setup.bash; ros2 launch robot_ur3e_perception real_yolov5_tf.launch.py
```

</details>

---

<details>
<summary><b>Resources</b></summary>

## Resources
Depth Camera Information:
- https://www.intelrealsense.com/depth-camera-d435i/

Virtual environments:
- https://medium.com/ros2-tips-and-tricks/running-ros2-nodes-in-a-python-virtual-environment-b31c1b863cdb

Parameters for camera:
- https://github.com/Marnonel6/YOLOv7_ROS2/blob/main/object_detection/launch/object_detection.launch.xml

</details>

---