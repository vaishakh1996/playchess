#!/usr/bin/env bash

# Remember to connect TIAGo to WiFi!
export ROS_MASTER_URI=http://tiago-108c:11311
export ROS_IP=10.68.0.${1:-128}	# 10.68.0.128 is the default IP address on this PC when
								# connected to TIAGo. A different number can be set as
								# command line argument when launching the script.
source devel/setup.bash