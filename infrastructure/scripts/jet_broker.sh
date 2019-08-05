#!/bin/bash

# Set up proper error handling and disallow unset arguments
set -o errexit -o errtrace -o pipefail -o nounset

source jet_functions.sh

function help () {
cat <<-END
Usage: jet broker

Starts up a new IPC Broker container. If one is already running, does nothing.

-h| --help           Show this message
-f| --force          Force restart of IPC Broker container
END
}

CONTAINER_NAME=IPCBroker

FORCE_RESTART=0
IPC_BROKER_IS_RUNNING=0
START=1

while [[ $# -gt 0 ]]; do
        case "$1" in
                -h|--help)
                        help
                        exit 0
                        ;;
                -f|--force*)
                        FORCE_RESTART=1
                        shift
                        ;;
                *)
                        break
                        ;;
        esac
done

if [[ $(docker ps | grep $CONTAINER_NAME) =~ $CONTAINER_NAME ]]; then
    IPC_BROKER_IS_RUNNING=1
    START=0
fi

if [[ $FORCE_RESTART -ne 0 ]]; then
    if [[ $IPC_BROKER_IS_RUNNING -ne 0 ]]; then
        docker stop $CONTAINER_NAME
        echo "Stopping existing $CONTAINER_NAME container."
    fi
    START=1
fi


if [[ $START -ne 0 ]]; then
    FULL_IMAGE_NAME=$(get_image_name)

    UNUSED=$(docker run -d --rm --name $CONTAINER_NAME --net=host --privileged $FULL_IMAGE_NAME mosquitto)

    if [ $? -ne 0 ]; then
        echo "Failed to start IPCBroker container."
        exit $?
    fi
    echo "Started IPCBroker container."
fi
