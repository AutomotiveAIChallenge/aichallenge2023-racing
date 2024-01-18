#!/bin/bash

export PATH="$PATH:/root/.local/bin"
export PATH="/usr/local/cuda/bin:$PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export XDG_RUNTIME_DIR=/tmp/xdg
export ROS_LOCALHOST_ONLY=1

# Launch AWSIM
echo "Launch AWSIM"
sudo ip link set multicast on lo
source /aichallenge/aichallenge_ws/install/setup.bash
/aichallenge/AWSIM/AWSIM.x86_64 &

sleep 20

cd /output

# Launch Autoware
echo "Launch user Autoware code"
source /aichallenge/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.xml > autoware.log 2>&1 &
ROSLAUNCH_PID=$!

sleep 10

# Record rosbag
ros2 bag record -a -o rosbag2_autoware &
ROSBAG_RECORD_PID=$!

# Waiting for Service Launch
echo "Waiting for Service Launch"
COUNT=0
until (ros2 service type /debug/service/capture_screen) 
do 
    if [ $COUNT -ge 36 ]; then
	echo "timeout count: $COUNT"
	break
    fi
    echo "Waiting for Service Launch"
    ((COUNT+=1))
    sleep 5
done

# Start Recording Rviz2
if ros2 service type /debug/service/capture_screen; then
    ros2 service call /debug/service/capture_screen std_srvs/srv/Trigger
fi

# Wait result.js
echo "Wait for result.json"
COUNT=0
until [ -f ~/awsim-logs/result.json ]
do
  if [ $COUNT -ge 240 ]; then
     echo "timeout count: $COUNT"
     break
  fi
  ((COUNT+=1))
  sleep 5
done

# Stop Recording Rviz2
if ros2 service type /debug/service/capture_screen; then
    ros2 service call /debug/service/capture_screen std_srvs/srv/Trigger
fi

sleep 10

## Save rosbag and log
kill $ROSBAG_RECORD_PID
kill $ROSLAUNCH_PID

sleep 10

## Compress rosbag
tar -czf rosbag2_autoware.tar.gz rosbag2_autoware

sleep 5

echo "Generation of result.json is completed."
cp ~/awsim-logs/result.json /output
cp ~/awsim-logs/verbose_result.json /output
# cp /autoware.log /output
# cp /rosbag2_autoware.tar.gz /output
