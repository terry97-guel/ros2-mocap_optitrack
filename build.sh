#!/bin/bash

# Source ROS Foxy setup script
source /opt/ros/foxy/setup.bash

# Initialize flags
REMOVE_FLAG=false
ALL_FLAG=false

# Parse command-line arguments
for arg in "$@"; do
	case $arg in
		--rm)
			REMOVE_FLAG=true
			shift
			;;
		--all)
			ALL_FLAG=true
			shift
			;;
		*)
			echo "Unknown option: $arg"
			exit 1
			;;
	esac
done

# Handle --rm flag
if [ "$REMOVE_FLAG" = true ]; then
	echo "INFO::removing exisiting build"
	rm -rf build
	rm -rf install
fi

# Handle --all flag
if [ "$ALL_FLAG" = true ]; then
	colcon build --packages-select mocap_optitrack_interfaces mocap_optitrack_client mocap_optitrack_w2b
	. install/setup.bash
else
	colcon build --packages-select mocap_optitrack_interfaces 
	. install/setup.bash
fi

# Launch the ROS2 application
ros2 launch launch/launch_y_up.py
