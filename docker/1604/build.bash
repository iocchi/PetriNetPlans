#!/bin/bash

# Use  ./build.bash [Dockerfile] [version]

IMAGENAME=ub1604_kinetic_pnp

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=0.1
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

docker build -t $IMAGENAME:$VERSION -f $DOCKERFILE .

