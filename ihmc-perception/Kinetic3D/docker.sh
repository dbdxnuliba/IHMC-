#!/bin/bash

IMAGE_TAG=kinetic3d:latest

# Assume dense_sptam node is inside a catkin workspace
CATKIN_WS_DIR=$PWD/../../

case "$1" in
  run)
    docker run -it \
    --privileged \
    --net=host \
    -v /etc/group:/etc/group:ro \
    -v /etc/passwd:/etc/passwd:ro \
    -v $CATKIN_WS_DIR:/usr/src/sptam \
    -v $HOME/.ssh:/home/$( id -u -n)/.ssh:ro \
    -v $SSH_AUTH_SOCK:/ssh-agent \
    --mount type=tmpfs,destination=/home/$( id -u -n) \
    -e "SSH_AUTH_SOCK=/ssh-agent" \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    $IMAGE_TAG \
    /bin/bash
    ;;
  build)
    docker build -t $IMAGE_TAG .
    ;;
  *)
    echo "Usage: $0 {run|build}"
    exit 1
esac