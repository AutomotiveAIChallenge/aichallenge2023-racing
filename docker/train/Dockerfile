FROM ghcr.io/automotiveaichallenge/aichallenge2023-racing/autoware-universe-no-cuda:latest

# To build full autoware repositories
RUN rm -rf /autoware/install/tier4_*_launch
RUN rm -rf /autoware/install/autoware_launch
# -----------------------------------

ENV ROS_LOCALHOST_ONLY 1
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
ENV export RCUTILS_COLORIZED_OUTPUT 1

RUN apt-get update
RUN apt-get -y install libgl1-mesa-glx libgl1-mesa-dri
RUN apt-get -y install net-tools