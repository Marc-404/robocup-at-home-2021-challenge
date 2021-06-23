FROM ros:melodic-perception

SHELL [ "/bin/bash", "-c" ]
RUN apt-get update; exit 0
RUN apt-get install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN curl -fsSL https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-key add -
RUN echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list

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
RUN apt-get update && apt-get install -y cmake g++ unzip libboost-all-dev \
    libopenblas-dev libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev \
    libhdf5-serial-dev protobuf-compiler the python-dev libgflags-dev \
    libgoogle-glog-dev liblmdb-dev python-pip ros-melodic-desktop-full \
    python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
    build-essential python-rosdep ros-melodic-roslisp-repl rapidjson-dev \
    automake libxerces-c-dev libicu-dev libapr1-dev mongodb-org openjdk-8-jdk \
    libatlas-base-dev liblapack-dev libblas-dev libmongoclient-dev \
    libgoogle-perftools4 libpcap0.8 libstemmer0d libtcmalloc-minimal4 \
    libeigen3-dev libmongoc-dev swi-prolog libjson-glib-dev libjson-glib-1.0-0 \
    ros-melodic-roslisp*

RUN sudo mkdir -p /data/db /data/configdb &&\
    sudo chown -R mongodb:mongodb /data/db /data/configdb

RUN pip install future protobuf pybullet==3.0.6 scipy==1.2.2 casadi sortedcontainers hypothesis==4.34.0 pandas==0.24.2 numpy==1.16 Parsetron rdflib

# install opencv
#RUN apt update
#RUN apt install -y python3-opencv

RUN apt install -y build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev 

RUN mkdir ~/opencv_build && cd ~/opencv_build && \
    git clone --branch 3.4 https://github.com/opencv/opencv.git && \
    git clone --branch 3.4 https://github.com/opencv/opencv_contrib.git && \
    cd opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF .. >> ../opencv_make && \
    make -j$(nproc) && make install

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
    make -j$(nproc) >> ../../make_output && \
    make install

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
RUN cd /workspace && /ros_entrypoint.sh rosdep install --from-paths src/suturo_navigation src/suturo_planning src/suturo_manipulation src/suturo_resources src/suturo_knowledge --ignore-src -r -y

# Download model data for classifier
RUN cd /workspace/src/rs_resources/extracted_feats && \
    wget -q -O BVLC_REF_ClassLabel_ycb_food.txt https://seafile.zfn.uni-bremen.de/f/6b2486588606451a8174/?dl=1 && \
    wget -q -O BVLC_REF_data_ycb_food.yaml https://seafile.zfn.uni-bremen.de/f/be4d1be7ef584d18b884/?dl=1 && \
    wget -q -O BVLC_REF_ClassLabel_ycb_kitchen.txt https://seafile.zfn.uni-bremen.de/f/160449d41839408bb081/?dl=1 && \
    wget -q -O BVLC_REF_data_ycb_kitchen.yaml https://seafile.zfn.uni-bremen.de/f/7d81d41a1dd745af838f/?dl=1 && \
    wget -q -O BVLC_REF_ClassLabel_ycb_shape.txt https://seafile.zfn.uni-bremen.de/f/9cd957f4a50142709fe4/?dl=1 && \
    wget -q -O BVLC_REF_data_ycb_shape.yaml https://seafile.zfn.uni-bremen.de/f/724d73afda2a4d8488d5/?dl=1 && \
    wget -q -O BVLC_REF_ClassLabel_ycb_task.txt https://seafile.zfn.uni-bremen.de/f/de0bcd12dc354b37b4ca/?dl=1 && \
    wget -q -O BVLC_REF_data_ycb_task.yaml https://seafile.zfn.uni-bremen.de/f/17fde82611544d80a9ca/?dl=1 && \
    wget -q -O BVLC_REF_ClassLabel_ycb_tool.txt https://seafile.zfn.uni-bremen.de/f/6ec44448f6604dd287c1/?dl=1 && \
    wget -q -O BVLC_REF_data_ycb_tool.yaml https://seafile.zfn.uni-bremen.de/f/f922ad4cd6ba4f65b941/?dl=1

RUN cd /workspace/src/rs_resources/caffe/models/bvlc_reference_caffenet && \
    wget -q -O bvlc_reference_caffenet.caffemodel https://seafile.zfn.uni-bremen.de/f/bc46160b29d840a0836e/?dl=1

# Add package to start everything
ADD . /workspace/src

# compile and install our algorithm
RUN cd /workspace && \
    /ros_entrypoint.sh catkin build -j1 robosherlock suturo_perception cram_language suturo_resources && \
    /ros_entrypoint.sh catkin build -j$(nproc)

# command to run the algorithm
CMD sudo mongod --fork --logpath /var/log/mongod.log; source /workspace/devel/setup.bash && roslaunch suturo_launch execute_order_66.launch
