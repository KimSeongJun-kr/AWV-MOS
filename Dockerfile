FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

# Install requirements
RUN apt-get update && apt-get install -y curl lsb-release gnupg g++ software-properties-common
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.0
RUN apt-get update && apt-get install -y libeigen3-dev libtbb-dev libpcl-dev libopencv-dev libgtsam-dev libgtsam-unstable-dev

# Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-ros-base ros-noetic-tf ros-noetic-pcl-conversions ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-rviz

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]