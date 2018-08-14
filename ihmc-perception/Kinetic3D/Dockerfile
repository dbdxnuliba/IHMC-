FROM introlab3it/rtabmap:latest
MAINTAINER Bhavyansh

# Basic tools
RUN apt-get update && \
	apt-get install -y locales apt-utils wget git openssh-client && \
	rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update
RUN apt-get install -y ros-kinetic-desktop-full
RUN rosdep init && rosdep update

# Locale magic
#RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
#    locale-gen en_US.UTF-8 && \
#    dpkg-reconfigure locales && \
#    /usr/sbin/update-locale LANG=en_US.UTF-8

# ROS robot-localization lib - needed by dense node
#RUN apt-get update && \
#	apt-get install -y ros-kinetic-robot-localization && \
#	rm -rf /var/lib/apt/lists/*


# Install libsuitesparse lib - needed by sptam node
RUN apt-get update && \
	apt-get install -y libsuitesparse-dev && \
	rm -rf /var/lib/apt/lists/*

# Install g2o lib - needed by sptam node
RUN git clone https://github.com/RainerKuemmerle/g2o.git && cd g2o/ && \
	git checkout -q 4b9c2f5b68d14ad479457b18c5a2a0bce1541a90 && \
	mkdir build && cd build && cmake ../ && make && make install && \
	cd ../../ && rm -rf g2o/

# Install catkin tool
# RUN apt-get update && \
#	apt-get install -y python-catkin-tools && \
#	rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y install nano libsqlite3-dev libpcl-dev libopencv-dev git cmake libproj-dev libqt5svg5-dev
RUN apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# Set ROS master URI as localhost is not found in /etc/hosts
ENV ROS_MASTER_URI "http://10.6.192.14:11311"

WORKDIR /usr/src/sptam/ros-env/Kinetic3D

CMD /bin/bash
