#!/bin/bash

function help () {
cat <<-END
Usage: jet build

Builds the jet project.

-h| --help           Show this message
END
}

while [ -n "$1" ]; do
    case "$1" in
        -h | --help)
            help
            exit
            ;;
        *)
            break
            ;;
    esac
done

JET_REPO_PATH=$(git rev-parse --show-toplevel)

if [ $? -ne 0 ]; then
    exit $?
fi

CPU_INFO=$(lscpu)
if [[ $(echo $CPU_INFO | grep "Architecture:") =~ "x86_64" ]]; then
    IMAGE_NAME="jet"
    J_NUM=12
fi
if [[ $(echo $CPU_INFO | grep "Architecture:") =~ "arm" ]]; then
    IMAGE_NAME="jet-arm"
    J_NUM="3"
fi
FULL_IMAGE_NAME=hoverjet/$IMAGE_NAME

TARGETS="${@:1}"

docker run -it -v $JET_REPO_PATH:/jet --net=host --privileged $FULL_IMAGE_NAME bash -c "cd /jet; cmake .; make -j $J_NUM $TARGETS;"
if [ $? -ne 0 ]; then
    exit $?
fi
