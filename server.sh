#!/bin/env bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

SSH_PORT=22
SSH_KEY_FILE="$HOME/.ssh/id_rsa.pub"
ROS_DISTRO=noetic
IMAGE_NAME=epan/tiago:"${ROS_DISTRO}"
USERNAME=pal

PUBLIC_KEY=$(cat "${SSH_KEY_FILE}")
USER_ID=$(id -u "${USER}")
GROUP_ID=$(id -g "${USER}")
CONTAINER_NAME="${IMAGE_NAME//[\/.]/-}"
CONTAINER_NAME="${CONTAINER_NAME/:/-}-ssh"

docker stop "$CONTAINER_NAME" >/dev/null 2>&1
docker rm --force "$CONTAINER_NAME" >/dev/null 2>&1

echo "Starting background container with access port ${SSH_PORT} for user ${USERNAME}"

COMMAND_FLAGS=()
COMMAND_FLAGS+=(--key "${PUBLIC_KEY}")
COMMAND_FLAGS+=(--user "${USERNAME}")
COMMAND_FLAGS+=(--uid "${USER_ID}")
COMMAND_FLAGS+=(--gid "${GROUP_ID}")

RUN_FLAGS=()
RUN_FLAGS+=(--device=/dev/dri:/dev/dri)
RUN_FLAGS+=(--env DISPLAY="${DISPLAY}")
RUN_FLAGS+=(--volume="${SCRIPT_DIR}/ros:/home/pal/.ros")
RUN_FLAGS+=(--volume="${SCRIPT_DIR}/workspace:/home/pal/ros_ws")
RUN_FLAGS+=(--volume="${SCRIPT_DIR}/share:/home/pal/share")
RUN_FLAGS+=(--volume=/dev/bus/usb:/dev/bus/usb)
RUN_FLAGS+=(--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw)
RUN_FLAGS+=(--privileged)
RUN_FLAGS+=(--ipc=host)
RUN_FLAGS+=(--network=host)

docker run -d --rm --cap-add sys_ptrace    \
  --user root                              \
  --name "${CONTAINER_NAME}"               \
  --hostname "${CONTAINER_NAME}"           \
  --add-host="${CONTAINER_NAME}:127.0.1.1" \
  "${RUN_FLAGS[@]}"                        \
  "${IMAGE_NAME}" /sshd_entrypoint.sh "${COMMAND_FLAGS[@]}"

echo "${CONTAINER_NAME}"