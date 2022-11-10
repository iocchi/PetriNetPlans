#!/bin/bash

# Use  ./build.bash [Dockerfile] [forcebuildtag]

IMAGENAME=iocchi/pnp

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

PNP_VERSION=`cat ../../version.txt`
GRPC_VERSION=1.48.2

FORCEBUILDTAG=""
if [ ! "$2" == "" ]; then
  FORCEBUILDTAG="--build-arg FORCEBUILD=$2"
fi

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"
GRPC="--build-arg GRPC_VERSION=$GRPC_VERSION"

echo "======================================="
echo "   Building $IMAGENAME:$VERSION "
echo "======================================="

docker build $UPAR -t $IMAGENAME:melodic-base -f Dockerfile.base . && \
docker build $GRPC -t $IMAGENAME:melodic-grpc -f $DOCKERFILE.grpc . && \
docker build $FORCEBUILDTAG -t $IMAGENAME:melodic-$PNP_VERSION -f $DOCKERFILE .

docker tag $IMAGENAME:melodic-$PNP_VERSION $IMAGENAME:melodic

