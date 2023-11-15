#!/bin/bash

export PATH="$PATH:/root/.local/bin"
export PATH="/usr/local/cuda/bin:$PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export XDG_RUNTIME_DIR=/tmp/xdg
export ROS_LOCALHOST_ONLY=1

# Launch AWSIM
echo "Launch AWSIM"
source /aichallenge/aichallenge_ws/install/setup.bash
/aichallenge/AWSIM/AWSIM.x86_64 &

sleep 10

cd /output

echo "Launch user code"
source /aichallenge/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.xml &

# Wait result.js
echo "Wait for result.json."
until [ -f /aichallenge/result.json ]
do
  sleep 5
done

echo "Generation of result.json is completed."
cp /aichallenge/result.json /output
