# Autopilot Differential-Drive Robot in Carla and ROS
### Run:
1. start carla server
`cd /opt/carla-simulator
./CarlaUE4.sh -RenderOffScreen -quality-level=Low`
or start schoomatic docker
`docker run --rm --gpus all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw klekkala/carla_schoomatic /bin/bash /LinuxNoEditor/CarlaUE4.sh -RenderOffScreen`
3. start carla client
`source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch timeout:=20000`
4. start navigation
`source ~/_carla-ros-bridge/catkin_ws/devel/setup.bash
roslaunch carla_navigation carla_navigation.launch`
