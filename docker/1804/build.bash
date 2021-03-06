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

docker build -t $IMAGENAME:base -f Dockerfile.base . && \
docker build -t $IMAGENAME:grpc -f $DOCKERFILE.grpc . && \
docker build $FORCEBUILDTAG -t ${IMAGENAME}:melodic-$VERSION -f $DOCKERFILE .

docker tag $IMAGENAME:melodic-$VERSION $IMAGENAME:latest

