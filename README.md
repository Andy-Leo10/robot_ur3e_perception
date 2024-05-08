# robot_ur3e_perception

- [robot\_ur3e\_perception](#robot_ur3e_perception)
  - [For searching a coffee space](#for-searching-a-coffee-space)
  - [For normal image](#for-normal-image)
  - [For depth image](#for-depth-image)

## For searching a coffee space
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception camera
```

## For normal image
```

cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception camera_rgb
```
## For depth image
```
cd ~/ros2_ws/ ;colcon build --packages-select robot_ur3e_perception;source install/setup.bash; ros2 run robot_ur3e_perception camera_mono
```