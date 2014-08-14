#!/bin/sh
make clean
cd ..
tar czvf PNPros.tgz --exclude=build --exclude=bin/* --exclude=CMakeFiles --exclude=*.cmake --exclude=*~ --exclude=.svn --exclude=Makefile --exclude=src_NOT_COMPLETE --exclude=makedist.sh PNPros

