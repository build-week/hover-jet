#!/bin/bash

JET_REPO_PATH=$(git rev-parse --show-toplevel)

if [ $? -ne 0 ]; then
    exit $?
fi

DATE=`date +%Y.%m.%d-%H.%M.%S`

echo Building image: hoverjet/jet:$DATE
docker build $JET_REPO_PATH/infrastructure/scripts/docker/ -t hoverjet/jet:$DATE
