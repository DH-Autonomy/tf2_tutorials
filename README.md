# Transform 2 (TF2) with ROS 2

## Motivation
While TF2 has been ported to ROS 2, the tutorials provided on TF2's website have not been ported to ROS 2's new RCLCPP API. This repository ports the examples provided within the TF2 tutorials to ROS 2, to provide concrete examples of how TF2 can be used.

## Building
`colcon build --packages-select tf2_tutorials`

### Part 1: Writing a TF2 Static Broadcaster
Uses turtlesim to show an example on how to broadcast transforms. To run it, use the executable built during colcon build.
`ros2 run tf2_tutorials broadcaster <insert_turtle_name>`

### Part 2: Writing a TF2 Static Listener
Uses turtlesim to show how a new turtle can follow an existing turtle through the use of TF transforms.

![Turtles Swimming After Each Other](https://github.com/DH-Autonomy/tf2_tutorials/blob/main/gif/tf2_listener.gif)

Steps to run:
1. `ros2 run turtlesim turtlesim_node`
2. `ros2 run tf2_tutorials broadcaster turtle1`
3. `ros2 run tf2_tutorials broadcaster turtle2`
4. `ros2 run tf2_tutorials listener turtle1 turtle2`

## In the pipeline...
Some tutorials haven't been neatly ported / ported at all:
1. frame_tf2_broadcaster
2. tf2, time and time_travelling
3. Python3
