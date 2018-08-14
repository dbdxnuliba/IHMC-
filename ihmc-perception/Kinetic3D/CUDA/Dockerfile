FROM ubuntu:16.04
MAINTAINER Jonathan Kosgei <jonathan@saharacluster.com>

# A docker container with the Nvidia kernel module and CUDA drivers installed

ENV CUDA_RUN https://developer.nvidia.com/compute/cuda/8.0/prod/local_installers/cuda_8.0.44_linux-run

RUN apt-get update && apt-get install -q -y \
  wget \
  module-init-tools \
  build-essential 

RUN cd /opt && \
  wget $CUDA_RUN && \
  chmod +x cuda_8.0.44_linux-run && \
  mkdir nvidia_installers && \
  ./cuda_8.0.44_linux-run -extract=`pwd`/nvidia_installers && \
  cd nvidia_installers && \
  ./NVIDIA-Linux-x86_64-367.48.run -s -N --no-kernel-module

RUN cd /opt/nvidia_installers && \
  ./cuda-linux64-rel-8.0.44-21122537.run -noprompt

# Ensure the CUDA libs and binaries are in the correct environment variables
ENV LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-8.0/lib64
ENV PATH=$PATH:/usr/local/cuda-8.0/bin

RUN cd /opt/nvidia_installers &&\
    ./cuda-samples-linux-8.0.44-21122537.run -noprompt -cudaprefix=/usr/local/cuda-8.0 &&\
    cd /usr/local/cuda/samples/1_Utilities/deviceQuery &&\ 
    make

WORKDIR /usr/local/cuda/samples/1_Utilities/deviceQuery
