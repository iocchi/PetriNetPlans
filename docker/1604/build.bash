#!/bin/bash

# Use  ./build.bash [Dockerfile] [forcebuildtag]

IMAGENAME=pnp  
IMAGENAMEOLD=pnp_1604_kinetic:latest  # back compatibility
IMAGENAMENEW=iocchi/pnp:kinetic


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

docker build -t $IMAGENAME:kinetic-base -f Dockerfile.base . && \
docker build $FORCEBUILDTAG -t $IMAGENAME:kinetic-$VERSION -f $DOCKERFILE .

docker tag $IMAGENAME:kinetic-$VERSION $IMAGENAMEOLD  # back compatibility

docker tag $IMAGENAME:kinetic-$VERSION $IMAGENAMENEW

