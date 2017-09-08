#!/bin/bash

# Author: Winston Liu
# Copyright 2017
# All rights reserved

# Arguments
SEC_PER_FRAME=1 # Number of seconds per frame extraction. Do not exceed camera fps.


### SETUP ###


# Check if another roscore is running.
if pgrep -x "roscore" > /dev/null; then
    echo "ERROR: Another roscore is running. Please kill all ROS programs before running this script."
    exit
fi

ERROR_MSG="
Run script using command:
    'export_rosbag_data.sh [ROSBAG_FILE_NAME] [EXPORT_START_TIME_IN_SECS] [EXPORT_ENDING_TIME_IN_SECS] [IMAGE_TOPIC]'
You can get the desired starting and ending times by viewing the rosbag in rqt."

if [[ "$#" -ne 4 ]]; then 
    echo "ERROR: Incorrect number of arguments."
    echo "$ERROR_MSG"
    exit
fi

# Check validity of arguments
RE_NUM='^[0-9]+$' # Check if argument is positive int. Decimals are not supported 
RE_BAG='\.bag$' # Check if string ends in .bag

if ! [[ "$1" =~ $RE_BAG  &&  "$2" =~ $RE_NUM  && "$3" =~ $RE_NUM ]]; then
    echo "ERROR: Arguments are of incorrect format."
    echo "$ERROR_MSG"
    exit
fi

# Sanity check to make sure ending time is greater than starting time
if (( $2 > $3 )); then
    echo "ERROR: Invalid ending time. Ending time is greater than starting time."
    exit
fi

# Extract bag file name
filename=$(basename ${1%.*})
# Create a folder using bag file name

echo "... Output folder will be created in $PWD"

folder_name="extract_${filename}_${2}_${3}_fr${SEC_PER_FRAME}"
if [ -d "$folder_name" ]; then
    echo "ERROR: Previous extracted folder exists. Please move or delete folder before continuing."
    exit
else
    mkdir "$folder_name"
fi


### MAIN PROGRAM ###



timediff=$(( $3 - $2 )) # Duration of segment to extract

# Start roscore
roscore & PID_ROS=$!
sleep 2 # Pause for a bit since there's usually a delay for roscore starting up

# Start rosbag playing
echo $1
rosbag play --clock --start=$2 --duration=$timediff $1 & PID_BAG=$!

# Need do decompress images before exporting
rosrun image_transport republish compressed in:=$4 out:=camera_out/image & PID_REPUBLISH=$!

cd "$folder_name"

rosrun image_view extract_images _filename_format:="frame%04i.jpg" _sec_per_frame:=$SEC_PER_FRAME image:=camera_out/image & PID_EXTRACT=$!

# Wait for rosbag to finish playing
trap "kill $PID_BAG 2> /dev/null" EXIT
while kill -0 $PID_BAG 2> /dev/null; do
    sleep 1
done

# stop processes
kill $PID_EXTRACT
kill $PID_REPUBLISH
kill $PID_ROS

cd ..

echo "... Done!"
