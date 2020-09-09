#!/bin/bash

# Use  ./build.bash [Dockerfile] [version]

IMAGENAME=ub1804_melodic_pnp

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=1.0
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

docker build --network=host -t $IMAGENAME:base -f Dockerfile.base . && \
docker build --network=host -t $IMAGENAME:$VERSION -f $DOCKERFILE .

