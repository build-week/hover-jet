#!/bin/bash

# Set up proper error handling and disallow unset arguments
set -o errexit -o errtrace -o pipefail -o nounset

source jet_functions.sh

function help () {
cat <<-END
Usage: jet run COMMAND

Starts a Docker container based on the jet image, then executes the specified command inside of it. If no command is specified, bash will be run.

-h| --help           Show this message
-b| --broker         Change the IP address and port of the broker
-q| --qemu-arm       Run in an arm container on x86_64
END
}

MQTT_ADDRESS=tcp://localhost:1883

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h | --help)
            help
            exit
            ;;
        -b | --broker)
                MQTT_ADDRESS=$2
                shift
                shift
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

jet_broker.sh

JET_REPO_PATH=$(git rev-parse --show-toplevel)

if [ $? -ne 0 ]; then
    exit $?
fi

FULL_IMAGE_NAME=$(get_image_name)

QEMU_ARM_MOUNT=""
if [[ "$ENABLE_QEMU_ARM" == 1 ]]; then
  QEMU_ARM_MOUNT="-v $(get_qemu_arm_static_path):$(get_qemu_arm_static_path)"
fi

ATTACH_WEBCAM_DEVICE=""
WEBCAM_DEFAULT_DEV_PATH=/dev/video0
if [ -d "$WEBCAM_DEFAULT_DEV_PATH" ]; then
  ATTACH_WEBCAM_DEVICE=--device=/dev/video0:/dev/video0
fi

WEBCAM_UUID_PATH="/dev/v4l"

xhost +

CONTAINER_ID=$( \
    docker run -it -d -v $JET_REPO_PATH:/jet \
    -v $WEBCAM_UUID_PATH:$WEBCAM_UUID_PATH \
    -v /logs:/logs \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    $QEMU_ARM_MOUNT \
    -e LOG_BASE_PATH=/logs/ \
    -e DISPLAY \
    -e NO_AT_BRIDGE=1 \
    -e MQTT_ADDRESS=$MQTT_ADDRESS \
    $ATTACH_WEBCAM_DEVICE \
    --net=host \
    --privileged $FULL_IMAGE_NAME bash \
)

if [ $? -ne 0 ]; then
    exit $?
fi

xauth list | while read line; do docker exec -it $CONTAINER_ID xauth add $line; if [ $? -ne 0 ]; then
    echo '"^ Not actually an error" - Ben'
fi; done

if [ -z "${@:1}" ]
then
    docker exec -it $CONTAINER_ID bash
else
    COMMAND="${@:1}"
    echo "Running command \"$COMMAND\" in container."
    docker exec -it $CONTAINER_ID bash -c "$COMMAND"
fi

docker stop $CONTAINER_ID
