#!/bin/bash

# Use  ./build.bash [Dockerfile] [forcebuildtag]

IMAGENAME=iocchi/pnp

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=`cat ../../version.txt`

FORCEBUILDTAG=""
if [ ! "$2" == "" ]; then
  FORCEBUILDTAG="--build-arg FORCEBUILD=$2"
fi

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"


echo "======================================="
echo "   Building $IMAGENAME:$VERSION "
echo "======================================="

docker build $UPAR -t $IMAGENAME:base -f Dockerfile.base . && \
docker build -t $IMAGENAME:grpc -f $DOCKERFILE.grpc . && \
docker build $FORCEBUILDTAG -t ${IMAGENAME}:noetic-$VERSION -f $DOCKERFILE .

docker tag $IMAGENAME:noetic-$VERSION $IMAGENAME:noetic
docker tag $IMAGENAME:noetic-$VERSION $IMAGENAME:latest

