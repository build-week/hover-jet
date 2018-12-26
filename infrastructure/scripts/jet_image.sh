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

DATE=`date +%Y.%m.%d-%H.%M.%S`

FULL_IMAGE_NAME_TAG="hoverjet/$IMAGE_NAME:$DATE"

echo Building image: $FULL_IMAGE_NAME_TAG
docker build $JET_REPO_PATH/infrastructure/scripts/docker/ -t $FULL_IMAGE_NAME_TAG
