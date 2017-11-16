#!/bin/bash

# Author: Winston Liu
# Copyright 2017
# All rights reserved

ERROR_MSG="
Run script using command:
    'velodyne_packets_to_pointcloud.sh [ROSBAG_FILE_NAME] [CALIBRATION_FILE_NAME]'

EX. './velodyne_packets_to_pointcloud.sh test.bag calibration.yaml'"

if [[ "$#" -ne 2 ]]; then 
    echo "ERROR: Incorrect number of arguments."
    echo "$ERROR_MSG"
    exit
fi

# Location of the script
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Find workspace containing the 'velodyne_pointcloud' folder, excluding Autoware folder
velodyne_dir=$(find ~ -regex '^.*\/src.*\/velodyne\/velodyne_pointcloud$' | grep -v Autoware)

if [ -z $velodyne_dir ]; then
    echo "ERROR: Could not find velodyne_pointcloud package. Please make sure it is in a catkin workspace and try again. Exiting ..."
    exit
fi

# Extract the catkin workspace directory (assumes that velodyne_pointcloud is in a catkin workspace)
workspace_dir=$(sed "s/src.*//" <<< "${velodyne_dir}")
setup_bash=${workspace_dir}/devel/setup.bash

if [ ! -f $setup_bash ]; then
    echo "ERROR: Could not find setup.bash file in velodyne_pointcloud catkin workspace. Exiting ..."
    exit
fi

source $setup_bash

### MAIN ###


# Output is put into `output.bag`
ROS_HOME=`pwd` rosrun velodyne_pointcloud bagconvert -b $1 -c $2
python ${script_dir}/bagmerge.py $1 output.bag

# Do some cleanup
rm rospack_cache_*
rm output.bag

echo "Done!"
