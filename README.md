# Autonomous Driving in Carla and ROS
### Run:
1. start carla server
`cd /opt/carla-simulator
./CarlaUE4.sh -RenderOffScreen -quality-level=Low`
2. start carla client
`source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch timeout:=20000`
3. start control message translator
`source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
roslaunch carla_twist_to_control carla_twist_to_control.launch`
4. start controller
`source ~/catkin_ws/devel/setup.bash
rosrun carla_navigation controller.py`
5. start planner
`source ~/catkin_ws/devel/setup.bash
rosrun carla_navigation planner.py`
### To be implemented:
1. Lauch files that easily lauches everything
2. Debugging
3. Differential wheeled robot
