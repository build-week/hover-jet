#!/bin/bash

JET_REPO_PATH=$(git rev-parse --show-toplevel)

if [ $? -ne 0 ]; then
    exit $?
fi

CPU_INFO=$(lscpu)
if [[ $(echo $CPU_INFO | grep "Architecture:") =~ "x86_64" ]]; then
    IMAGE_NAME="jet"
fi
if [[ $(echo $CPU_INFO | grep "Architecture:") =~ "arm" ]]; then
    IMAGE_NAME="jet-arm"
fi
FULL_IMAGE_NAME=hoverjet/$IMAGE_NAME

docker run -it -v $JET_REPO_PATH:/jet --net=host --privileged $FULL_IMAGE_NAME bash -c "cd /jet; cmake .; make;"
if [ $? -ne 0 ]; then
    exit $?
fi
