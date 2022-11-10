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

echo "======================================="
echo "   Building $IMAGENAME:$VERSION "
echo "======================================="

docker build -t $IMAGENAME:melodic-base -f Dockerfile.base . && \
docker build -t $IMAGENAME:melodic-grpc -f $DOCKERFILE.grpc . && \
docker build $FORCEBUILDTAG -t ${IMAGENAME}:melodic-$VERSION -f $DOCKERFILE .

docker tag $IMAGENAME:melodic-$VERSION $IMAGENAME:melodic
#docker tag $IMAGENAME:melodic-$VERSION $IMAGENAME:latest

