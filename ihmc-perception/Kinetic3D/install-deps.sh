#! /bin/bash

# Install ros-utils lib - needed by sptam node
git clone https://github.com/lrse/ros-utils.git && cd ros-utils/ && \
        mkdir build && cd build && cmake ../ && make && make install && \
        cd ../../ && rm -rf ros-utils/

# Install DLib lib - needed by sptam node
git clone https://github.com/dorian3d/DLib.git && cd DLib/ && \
        git checkout -q 70089a38056e8aebd5a2ebacbcb67d3751433f32 && \
        mkdir build && cd build && cmake ../ && make && make install && \
        cd ../../ && rm -rf DLib/

# Install DBoW2 lib - needed by sptam node
git clone https://github.com/dorian3d/DBoW2.git && cd DBoW2/ && \
        git checkout -q 82401cad2cfe7aa28ee6f6afb01ce3ffa0f59b44 && \
        mkdir build && cd build && cmake ../ && make && make install && \
        cd ../../ && rm -rf DBoW2/

# Install DLoopDetector lib - needed by sptam node
git clone https://github.com/dorian3d/DLoopDetector.git && cd DLoopDetector/ && \
        git checkout -q 8e62f8ae84d583d9ab67796f779272b0850571ce && \
        mkdir build && cd build && cmake ../ && make && make install && \
        cd ../../ && rm -rf DLoopDetector/

# Install opengv lib - needed by sptam node
git clone https://github.com/laurentkneip/opengv.git && cd opengv/ && \
        git checkout -q 2e2d21917fd2fb75f2134e6d5be7a2536cbc7eb1 && \
        mkdir build && cd build && cmake ../ && make && make install && \
        cd ../../ && rm -rf opengv/

rm -rf /usr/src/gtest/build && mkdir /usr/src/gtest/build && cd /usr/src/gtest/build && cmake .. -DBUILD_SHARED_LIBS=ON && make && cd - && cp /usr/src/gtest/build/*.so /usr/lib


# Building the S-PTAM source code use one of the following two options:
# 	First catkin_make command does not include loop closures
# 	Second catkin_make comand includes the loop closure features (Default)

# catkin_make --pkg sptam -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSINGLE_THREAD=OFF -DSHOW_TRACKED_FRAMES=ON -DSHOW_PROFILING=ON -DPARALLELIZE=ON
catkin_make --pkg sptam -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_LOOPCLOSURE=ON -DSINGLE_THREAD=OFF -DSHOW_TRACKED_FRAMES=ON -DSHOW_PROFILING=ON -DPARALLELIZE=ON
