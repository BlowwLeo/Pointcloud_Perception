FROM nvcr.io/nvidia/tensorrt:23.07-py3
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update
RUN apt install sudo

RUN sudo apt update && sudo apt install locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

#Install ros
RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository universe -y
RUN sudo apt update && sudo apt install curl -y

RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo apt update
RUN sudo DEBIAN_FRONTEND=noninteractive apt install ros-humble-ros-base -y
RUN sudo apt install ros-dev-tools -y


#Install librairies
RUN sudo apt -y install ament-cmake \
    build-essential \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-test-dev \
    libgmp-dev \
    libmpfr-dev \
    libcgal-dev \
    libcgal-demo \
    libopencv-dev \
    librange-v3-dev \
    libpugixml-dev \
    libgeographic-dev \
    libopenmpi-dev


#Install ros-humble librairies
RUN sudo apt-get -y install ros-humble-lanelet2-core \
    ros-humble-lanelet2-io \
    ros-humble-lanelet2-routing \
    ros-humble-lanelet2-projection \
    ros-humble-cudnn-cmake-module \
    ros-humble-tensorrt-cmake-module \
    ros-humble-point-cloud-msg-wrapper \
    ros-humble-image-transport \
    ros-humble-diagnostic-updater \
    ros-humble-cv-bridge \
    ros-humble-pcl-ros \
    ros-humble-logging-demo

RUN sudo apt update -y


#Copy Pointcloud_Perception package
RUN mkdir -p Pointcloud_Perception/src
RUN mkdir -p Pointcloud_Perception/data
RUN mkdir -p Pointcloud_Perception/launch

COPY src Pointcloud_Perception/src
COPY data Pointcloud_Perception/data
COPY launch Pointcloud_Perception/launch
COPY eigen3 .

#Build packages
WORKDIR /workspace/Pointcloud_Perception/
RUN /bin/bash -c "source /opt/ros/humble/setup.bash ;\
    colcon build --parallel-workers 1"
