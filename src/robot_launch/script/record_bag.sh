#!/bin/bash

bag_name=/home/nvidia/leapting_robot/script/robot_view/bag/record_bag
function start_record() {
  echo "Start recording..."
  topic_info=$(rostopic info /rslidar_points)
  if [ -z "$topic_info" ]; then
    echo "The /rslidar_points topic is not available. Please check the topic and try again."
    exit 1
  fi
  rosbag record /imu /rslidar_points /tf -O $bag_name.bag &
}

function stop_record() {
  echo "Stop recording..."
  pids=$(pgrep -f "rosbag/record")
  if [ -n "$pids" ]; then
    echo "Terminating rosbag record processes: $pids"
    echo 'nvidia' | sudo -S  kill -9 $pids
  else
    echo "No rosbag record processes found"
  fi

  if [ -f "$bag_name.bag.active" ]; then
    echo "Detected active bag file. Performing repair operation..."
    rosbag reindex $bag_name.bag.active
    rosbag fix  $bag_name.bag.active  $bag_name.bag
    rm  $bag_name.bag.orig.active
    rm  $bag_name.bag.active
    echo "Repair operation complete."
else
    echo "No active bag file detected."
fi
}


if [ "$1" = "record" ]; then
  echo "start recording ......"
  rm $bag_name.bag
  start_record
elif [ "$1" = "end" ]; then
  stop_record
else
  echo "Invalid argument. Please use 'record' to start recording or 'end' to stop."
fi