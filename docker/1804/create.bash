#!/bin/bash

# Use  ./run.bash [version]

IMAGENAME=pnp_1804_melodic

VERSION=latest
if [ ! "$1" == "" ]; then
  VERSION=$1
fi

# change setings here if needed
PLAYGROUND_FOLDER=$HOME/playground

echo "Running image $IMAGENAME:$VERSION ..."

docker create -it \
    --name petri_net_plans \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    -e DISPLAY=$DISPLAY \
    --privileged \
    --net=host \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    $IMAGENAME:$VERSION

