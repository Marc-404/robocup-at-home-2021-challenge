FROM ros:melodic-perception

SHELL [ "/bin/bash", "-c" ]
RUN apt-get update; exit 0
RUN apt-get install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# install depending packages (install moveit! algorithms on the workspace side, since moveit-commander loads it from the workspace)
RUN apt-get update && \
    apt-get install -y git ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-move-base-msgs ros-$ROS_DISTRO-ros-numpy ros-$ROS_DISTRO-geometry && \
    apt-get clean

# install bio_ik
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p /bio_ik_ws/src && \
    cd /bio_ik_ws/src && \
    catkin_init_workspace && \
    git clone --depth=1 https://github.com/TAMS-Group/bio_ik.git && \
    cd .. && \
    catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 && \
    cd / && rm -r /bio_ik_ws

# install dependencies for build
RUN apt install -y wget git python-pip python3-pip tree
RUN apt install -y python-rosinstall
RUN apt install -y python-wstool
RUN apt install -y python-catkin-tools
RUN pip install pytess

# create workspace folder
RUN mkdir -p /workspace/src

# clone our repos
RUN cd /workspace/src
RUN cd /workspace/src && wstool init
RUN cd /workspace/src && wstool merge https://raw.githubusercontent.com/SUTURO/suturo_resources/robocup/rosinstall.rosinstall -y
#RUN cd /workspace/src && wstool merge https://raw.githubusercontent.com/SUTURO/suturo_planning/robocup/planning_dependency.rosinstall -y
#RUN cd /workspace/src && wstool merge https://raw.githubusercontent.com/SUTURO/suturo_perception/robocup/dependencies.rosinsall -y
#RUN cd /workspace/src && wstool merge https://raw.githubusercontent.com/SUTURO/suturo_knowledge/robocup/dependencies.rosinstall -y
RUN cd /workspace/src && wstool merge https://raw.githubusercontent.com/SUTURO/suturo_navigation/robocup/dependencies.rosinstall -y
#RUN cd /workspace/src && wstool merge https://raw.githubusercontent.com/SUTURO/suturo_manipulation/robocup/dependencies.rosinstall -y

RUN wstool merge https://raw.githubusercontent.com/SUTURO/suturo_navigation/robocup/workspace.rosinstall -y

RUN cd /workspace/src && wstool update

# install dependencies defined in package.xml
RUN cd /workspace && /ros_entrypoint.sh rosdep install --from-paths src --ignore-src -r -y

# compile and install our algorithm
RUN cd /workspace/src && tree
RUN cd /workspace && /ros_entrypoint.sh catkin build

# command to run the algorithm
CMD source /workspace/devel/setup.bash && roslaunch suturo_bringup robocup_bringup.launch
