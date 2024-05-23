rm -rf build
rm -rf install

source /opt/ros/foxy/setup.bash
colcon build --packages-select mocap_optitrack_interfaces mocap_optitrack_client mocap_optitrack_w2b
. install/setup.bash

ros2 launch launch/launch_y_up.py