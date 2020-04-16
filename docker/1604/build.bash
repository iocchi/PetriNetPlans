#!/bin/bash

# Use  ./build.bash [Dockerfile] [version]

IMAGENAME=ub1604_kinetic_pnp

docker build -t $IMAGENAME:base -f Dockerfile.base .

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=0.1
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

docker build --no-cache -t $IMAGENAME:$VERSION -f $DOCKERFILE .

