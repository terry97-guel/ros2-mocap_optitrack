#!/bin/bash

# Source ROS Foxy setup script
source /opt/ros/foxy/setup.bash

# Initialize flags
LOCAL_FLAG=false
ONBOARD_FLAG=false

# Parse command-line arguments
for arg in "$@"; do
	case $arg in
		--l)
			LOCAL_FLAG=true
			shift
			;;
		--b)
			ONBOARD_FLAG=true
			shift
			;;
		*)
			echo "Unknown option: $arg"
			exit 1
			;;
	esac
done

# Handle --rm flag
# if [ "$REMOVE_FLAG" = true ]; then
# 	echo "INFO::removing exisiting build"
# 	rm -rf build
# 	rm -rf install
# 	rm -rf log
# fi

# colcon build --packages-select mocap_optitrack_interfaces mocap_optitrack_client
# . install/setup.bash

if [ "$ONBOARD_FLAG" = true ]; then
	rm -rf build
	rm -rf install
	rm -rf log

	# Touching solves timing problem somehow (?)
	find /opt/ros/ -exec touch {} +
	find /usr/include/python3.8 -exec touch {} +
	find /home/isaac/go1_gym -exec touch {} +
	colcon build --packages-select mocap_optitrack_interfaces 
	. install/setup.bash
fi

if [ "$LOCAL_FLAG" = true ]; then
	colcon build --packages-select mocap_optitrack_interfaces mocap_optitrack_client
	. install/setup.bash
	# Launch the ROS2 application
	ros2 launch launch/launch_mocap.py
fi

echo "INFO:: sourcing" # Somehow sourcing does not work in onboard docker 
source /opt/ros/foxy/setup.bash
. install/setup.bash