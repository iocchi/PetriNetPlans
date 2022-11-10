#!/bin/bash

# Use  ./build.bash [Dockerfile] [forcebuildtag]

IMAGENAME=iocchi/pnp

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

PNP_VERSION=`cat ../../version.txt`
GRPC_VERSION=1.50.0

FORCEBUILDTAG=""
if [ ! "$2" == "" ]; then
  FORCEBUILDTAG="--build-arg FORCEBUILD=$2"
fi

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"
GRPC="--build-arg GRPC_VERSION=$GRPC_VERSION"

echo "======================================="
echo "   Building $IMAGENAME:$VERSION "
echo "======================================="

docker build $UPAR -t $IMAGENAME:noetic-base -f Dockerfile.base . && \
docker build $GRPC -t $IMAGENAME:noetic-grpc -f $DOCKERFILE.grpc . && \
docker build $FORCEBUILDTAG -t $IMAGENAME:noetic-$PNP_VERSION -f $DOCKERFILE .

docker tag $IMAGENAME:noetic-$PNP_VERSION $IMAGENAME:noetic
docker tag $IMAGENAME:noetic-$PNP_VERSION $IMAGENAME:latest

