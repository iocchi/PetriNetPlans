#!/bin/bash

# Use  ./build.bash [Dockerfile] [forcebuildtag]

IMAGENAME=pnp_kinetic

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
docker build $FORCEBUILDTAG -t $IMAGENAME:$VERSION -f $DOCKERFILE .

docker tag $IMAGENAME:$VERSION $IMAGENAME:latest
docker tag $IMAGENAME:$VERSION pnp_1604_kinetic:latest  # back compatibility

