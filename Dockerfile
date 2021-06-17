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
RUN apt-get install -y cmake g++ unzip libboost-all-dev libopenblas-dev libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler the python-dev libgflags-dev libgoogle-glog-dev liblmdb-dev python-pip python3-pip ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep ros-melodic-roslisp-repl rapidjson-dev automake libxerces-c-dev libicu-dev libapr1-dev mongodb openjdk-8-jdk libatlas-base-dev liblapack-dev libblas-dev libmongoclient-dev
RUN pip install future protobuf tinyrpc==0.9.4 pyzmq pybullet==3.0.6 scipy==1.2.2 casadi sortedcontainers hypothesis==4.34.0 pandas==0.24.2 numpy==1.16
RUN pip3 install simplenlg http://www.jbox.dk/sling/sling-2.0.0-py3-none-linux_x86_64.whl tinyrpc==0.9.4 pyzmq


#install caffe
#RUN apt install -y caffe-cpu

RUN mkdir /caffe && cd /caffe && \
    wget -q -O caffe.zip https://github.com/BVLC/caffe/archive/master.zip && \
    unzip -qq caffe.zip && \
    rm caffe.zip && \
    cd caffe-master && \
    wget -q -O Makefile.config https://raw.githubusercontent.com/SUTURO/suturo_ci/master/setup/caffe/Makefile.config && \
    mkdir build && cd build && \
    cmake .. >> ../../cmake_output && \  
    #wget -q -O CaffeConfig.cmake https://raw.githubusercontent.com/SUTURO/suturo_perception/robocup/CaffeConfig.cmake && \
    wget -q -O cmake_install.cmake https://raw.githubusercontent.com/SUTURO/suturo_perception/robocup/cmake_install.cmake && \
    make -j all >> ../../make_output && \
    make install


# install opencv
#RUN apt update
RUN apt install -y python3-opencv
#RUN mkdir /opencv && cd /opencv && \
#    wget -q -O opencv4_5_1.tar.gz https://github.com/opencv/opencv/archive/4.5.1.tar.gz && \
#    wget -q -O opencv_contrib_4_5_1.tar.gz https://github.com/opencv/opencv_contrib/archive/4.5.1.tar.gz && \
#    tar xfz opencv4_5_1.tar.gz && \
#    tar xfz opencv_contrib_4_5_1.tar.gz && \
#    rm opencv4_5_1.tar.gz opencv_contrib_4_5_1.tar.gz && \
#    cd opencv-4.5.1 && \
#    mkdir build && cd build && \
#    cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.5.1/modules .. >> ../../cmake_output && \
#    make -j$(nproc) >> ../../make_output && \
#    make install


# create workspace folder
RUN mkdir -p /workspace/src

# clone our repos
RUN cd /workspace/src && \
    wstool init && \
    wstool merge https://raw.githubusercontent.com/cram2/cram/master/cram-18.04.rosinstall -y && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_resources/robocup/rosinstall.rosinstall -y && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_planning/robocup/planning_dependency.rosinstall -y && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_perception/robocup/dependencies.rosinsall -y && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_knowledge/robocup/dependencies.rosinstall -y && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_navigation/robocup/dependencies.rosinstall -y && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_manipulation/robocup/dependencies.rosinstall -y


RUN cd /workspace/src && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_navigation/robocup/workspace.rosinstall -y && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_manipulation/robocup/workspace.rosinstall -y && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_perception/robocup/workspace.rosinstall -y && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_knowledge/robocup/workspace.rosinstall -y && \
    wstool merge https://raw.githubusercontent.com/SUTURO/suturo_planning/robocup/planning_ws.rosinstall -y

RUN cd /workspace/src && wstool update

# install dependencies defined in package.xml
RUN cd /workspace && /ros_entrypoint.sh rosdep install --from-paths src --ignore-src -r -y

# Add package to start everything
ADD . /workspace/src

# compile and install our algorithm
RUN cd /workspace && \
    /ros_entrypoint.sh catkin build -j1 robosherlock suturo_perception && \
    /ros_entrypoint.sh catkin build

# command to run the algorithm
CMD source /workspace/devel/setup.bash && roslaunch suturo_launch execute_order_66.launch
