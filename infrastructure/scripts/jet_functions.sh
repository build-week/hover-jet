#!/bin/bash

get_image_name() {
    # The bash lines below are left intentionally commented
    # They will be returned when it is time to return to the latest Docker image
    CPU_INFO=$(lscpu)
    if [[ $(echo $CPU_INFO | grep "Architecture:") =~ "x86_64" ]]; then
        IMAGE_NAME="hoverjet/jet"
    fi
    if [[ $(echo $CPU_INFO | grep "Architecture:") =~ "arm" ]]; then
        IMAGE_NAME="hoverjet/jet-arm"
    fi
    if [[ $(echo $CPU_INFO | grep "Architecture:") =~ "aarch64" ]]; then
        IMAGE_NAME="hoverjet/jet-aarch64"
    fi
    echo "$IMAGE_NAME"
}
