#!/bin/bash

# Use  ./build.bash [Dockerfile] [version]

IMAGENAME=iocchi/pnp

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=`cat ../../version.txt`
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

echo "======================================="
echo "   Building $IMAGENAME:$VERSION "
echo "======================================="

docker build --network=host -t $IMAGENAME:base -f Dockerfile.base . && \
docker build --network=host -t $IMAGENAME:grpc -f $DOCKERFILE.grpc . && \
docker build --network=host -t ${IMAGENAME}:melodic-$VERSION -f $DOCKERFILE .

docker tag $IMAGENAME:melodic-$VERSION $IMAGENAME:latest

