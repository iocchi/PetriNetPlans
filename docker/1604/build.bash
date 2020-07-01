#!/bin/bash

# Use  ./build.bash [Dockerfile] [version]

IMAGENAME=ub1604_kinetic_pnp

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=1.0
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

docker build --no-cache -t $IMAGENAME:base -f Dockerfile.base . && \
    docker build --no-cache -t $IMAGENAME:$VERSION -f $DOCKERFILE .

