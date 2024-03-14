#!/bin/bash

sudo ip link set multicast on lo

source ./aichallenge_ws/install/setup.bash
/aichallenge/AWSIM/AWSIM.x86_64
