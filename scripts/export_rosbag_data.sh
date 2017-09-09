#!/bin/bash

# Author: Winston Liu
# Copyright 2017
# All rights reserved

# Arguments
FPS=15 # Default FPS of video
SEC_PER_FRAME=1 # Seconds per frame

OUTPUT_VIDEO_INSTEAD_OF_IMAGES=false

### SETUP ###


# Check if another roscore is running.
if pgrep -x "roscore" > /dev/null; then
    echo "ERROR: Another roscore is running. Please kill all ROS programs before running this script."
    exit
fi

ERROR_MSG="
Run script using command:
    'export_rosbag_data.sh [ROSBAG_FILE_NAME] [EXPORT_START_TIME_IN_SECS] [EXPORT_ENDING_TIME_IN_SECS] [CAMERA_TOPIC_NAME]'

EX. './export_rosbag_data.sh car_test2 362 388 ProcessedStereoToRos'
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

folder_name="extract_${filename}"
if ! [ -d "$folder_name" ]; then
    mkdir "$folder_name"
fi


### MAIN PROGRAM ###


# Some variables
bagstart_sec=$(temp=$(rosbag info -y -k start $1); echo ${temp%.*})
bagtime_filter_start=$(( $2 + $bagstart_sec ))
bagtime_filter_end=$(( $3 + $bagstart_sec ))
new_bag_name="filtered_${filename}_${2}_${3}.bag"

# Cut out the piece of the rosbag that we're interested in. This command is blocking
echo "Copying bag from ${bagtime_filter_start} to ${bagtime_filter_end}."
rosbag filter $1 $folder_name/$new_bag_name "t.to_sec() >= $bagtime_filter_start and t.to_sec() <= $bagtime_filter_end"

cd "$folder_name"

# Start roscore
roscore & PID_ROS=$!
sleep 2 # Pause for a bit since there's usually a delay for roscore starting up

# Start rosbag playing
echo $1
rosbag play --clock $new_bag_name & PID_BAG=$!

# Need do decompress images before exporting
rosrun image_transport republish compressed in:=${4}/left/image_rect out:=camera_out/image & PID_REPUBLISH=$!

# Run either the video recording or the image extraction based on the setting of the flag
if $OUTPUT_VIDEO_INSTEAD_OF_IMAGES; then
    rosrun image_view video_recorder _fps:=$FPS _filename:="${filename}_start_${2}_dur_${timediff}.avi" image:=camera_out/image & PID_EXTRACT=$!
else
    rosrun image_view extract_images _filename_format:="${filename}_fr%04i.jpg" _sec_per_frame:=$SEC_PER_FRAME image:=camera_out/image & PID_EXTRACT=$!
fi 

# Output camera info
rostopic echo -n 1 ${4}/left/camera_info > camera_info.txt

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
