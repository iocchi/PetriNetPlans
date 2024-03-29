#!/bin/bash

# Use  ./run.bash [version]

IMAGENAME=iocchi/pnp

VERSION=noetic
if [ ! "$1" == "" ]; then
  VERSION=$1
fi

# change setings here if needed
PLAYGROUND_FOLDER=$HOME/playground
PNP_FOLDER=$HOME/src/PetriNetPlans

echo "Running image $IMAGENAME:$VERSION ..."

docker run -it \
    --name pnp --rm \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    -e DISPLAY=$DISPLAY \
    --privileged \
    --net=host \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    -v $PNP_FOLDER:/home/robot/src/PetriNetPlans \
    $IMAGENAME:$VERSION


