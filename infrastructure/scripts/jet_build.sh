#!/bin/bash

source jet_functions.sh

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

FULL_IMAGE_NAME=$(get_image_name)
J_NUM=`expr $(nproc) - 1`

TARGETS="${@:1}"

docker run -it -v $JET_REPO_PATH:/jet --net=host --privileged $FULL_IMAGE_NAME bash -c "mkdir -p /jet/bin; cd /jet/bin; cmake ..; make -j $J_NUM $TARGETS;"
if [ $? -ne 0 ]; then
    exit $?
fi
