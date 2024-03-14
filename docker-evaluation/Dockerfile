FROM ghcr.io/automotiveaichallenge/aichallenge2023-racing/autoware-universe-no-cuda:latest

# Advance Preparation (Create working files and download various files from Github)

# To build full autoware repositories
RUN rm -rf /autoware/install/tier4_*_launch
RUN rm -rf /autoware/install/autoware_launch
# -----------------------------------

RUN mkdir /ws
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
RUN sudo apt install -y git-lfs
RUN git lfs clone https://github.com/AutomotiveAIChallenge/aichallenge2023-racing /ws/aichallenge2023-racing

# Copy into Container
COPY AWSIM /ws/AWSIM
COPY aichallenge_submit.tar.gz /ws
COPY main.bash /ws

# Organize Files for Execution (Copy user files and map data to the executable folder)
RUN cp -r /ws/aichallenge2023-racing/docker/aichallenge /aichallenge
RUN cp -r /ws/AWSIM /aichallenge
RUN chmod 757 /aichallenge
RUN rm -rf /aichallenge/aichallenge_ws/src/aichallenge_submit
RUN tar zxf /ws/aichallenge_submit.tar.gz -C /aichallenge/aichallenge_ws/src

# Build
RUN apt-get update
RUN apt-get -y install libgl1-mesa-glx libgl1-mesa-dri
RUN bash -c ' \
  source /autoware/install/setup.bash; \
  cd /aichallenge/aichallenge_ws; \
  rosdep update; \
  rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO; \
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

ENTRYPOINT []
CMD ["bash", "/ws/main.bash"]
