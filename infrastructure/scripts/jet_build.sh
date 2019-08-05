#!/bin/bash

# Set up proper error handling and disallow unset arguments
set -o errexit -o errtrace -o pipefail -o nounset

source jet_functions.sh

function help () {
cat <<-END
Usage: jet build

Builds the jet project.

-h| --help           Show this message
-q| --qemu-arm       Build for arm on x86_64
END
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h | --help)
            help
            exit
            ;;
        -q | --qemu-arm)
            export ENABLE_QEMU_ARM=1
            shift
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

QEMU_ARM_MOUNT=""
if [[ "$ENABLE_QEMU_ARM" == 1 ]]; then
  QEMU_ARM_MOUNT="-v $(get_qemu_arm_static_path):$(get_qemu_arm_static_path)"
fi

TARGETS="${@:1}"

docker run -it -v $JET_REPO_PATH:/jet $QEMU_ARM_MOUNT --net=host --privileged $FULL_IMAGE_NAME bash -c "mkdir -p /jet/bin; cd /jet/bin; cmake ..; make -j $J_NUM $TARGETS;"
if [ $? -ne 0 ]; then
    exit $?
fi
