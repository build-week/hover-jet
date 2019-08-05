#!/bin/bash

# Set up proper error handling and disallow unset arguments
set -o errexit -o errtrace -o pipefail -o nounset

ENABLE_QEMU_ARM=0

_echo_err() {

    # Send messages to the console on stderr since stdout is used for returns
    echo "$@" >&2
}

get_qemu_arm_static_path() {

    if [[ "$ENABLE_QEMU_ARM" == 1 ]]; then

        # Detect if the qemu-arm-static binary is installed and return the path if so
        echo "$(ls /usr/bin/qemu-arm-static 2> /dev/null)"
    fi
}

_get_architecture() {
    local cpu_info="$(lscpu | grep '^Architecture:')"
    local cpu_architecture=""

    if [[ "$cpu_info" =~ "x86_64" ]]; then
        cpu_architecture="x86_64"
    elif [[ "$cpu_info" =~ "arm" ]]; then
        cpu_architecture="arm"
    fi

    echo "$cpu_architecture"
}

_map_architecture_to_image() {
    local cpu_architecture="$1"
    local image_name=""

    if [[ "$cpu_architecture" == "x86_64" ]]; then
        image_name="hoverjet/jet"
    elif [[ "$cpu_architecture" == "arm" ]]; then
        image_name="hoverjet/jet-arm"
    fi

    echo "$image_name"
}

_qemu_arm_image() {
    if [[ "$(_get_architecture)" != "x86_64" ]]; then
        _echo_err "Using --qemu-arm is only intended for x86_64 platforms"
        exit 1
    elif [[ -z "$(get_qemu_arm_static_path)" ]]; then
        _echo_err "You need to install the qemu-user-static package first"
        _echo_err ""
        _echo_err "Run the following command to do so:"
        _echo_err "sudo apt-get install qemu-user-static"
        exit 1
    fi

    echo "hoverjet/jet-arm"
}

get_image_name() {
    if [[ "$ENABLE_QEMU_ARM" == 1 ]]; then
        _qemu_arm_image
    else
        _map_architecture_to_image "$(_get_architecture)"
    fi
}
