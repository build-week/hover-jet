#!/bin/bash

function help () {
cat <<-END
Usage: jet run COMMAND

Starts a docker container from the jet image, then executes the specified command inside of it. If no command is specified, bash will be run.

-h| --help           Show this message
END
}

MQTT_ADDRESS=tcp://localhost:1883

while [ -n "$1" ]; do
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

CPU_INFO=$(lscpu)
if [[ $(echo $CPU_INFO | grep "Architecture:") =~ "x86_64" ]]; then
    IMAGE_NAME="jet"
fi
if [[ $(echo $CPU_INFO | grep "Architecture:") =~ "arm" ]]; then
    IMAGE_NAME="jet-arm"
fi
FULL_IMAGE_NAME=hoverjet/$IMAGE_NAME

WEBCAM_DEFAULT_DEV_PATH=/dev/video0
if [ -d "$WEBCAM_DEFAULT_DEV_PATH" ]; then
  ATTACH_WEBCAM_DEVICE=--device=/dev/video0:/dev/video0
fi

xhost +

CONTAINER_ID=$(docker run -it -d -v $JET_REPO_PATH:/jet -v /logs:/logs -v /tmp/.X11-unix:/tmp/.X11-unix -e LOG_BASE_PATH=/logs/ -e DISPLAY -e NO_AT_BRIDGE=1 -e MQTT_ADDRESS=$MQTT_ADDRESS $ATTACH_WEBCAM_DEVICE --net=host --privileged $FULL_IMAGE_NAME bash)

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
