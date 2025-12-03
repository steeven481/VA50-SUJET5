#!/bin/bash

set -e

ROS_DISTRO=noetic
IMAGE_NAME=epan/tiago:"${ROS_DISTRO}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

BASE_IMAGE_NAME="palrobotics/tiago-devel-pc"
BASE_IMAGE_ARCHIVE="docker/image/TIAGo-development-computer-ubuntu-20.04.tar.gz"

if docker image inspect "$BASE_IMAGE_NAME" > /dev/null 2>&1; then
    echo "Docker image '$BASE_IMAGE_NAME' already exists. Skipping import."
else
    echo "Importing Docker image from $BASE_IMAGE_ARCHIVE..."
    docker import "$BASE_IMAGE_ARCHIVE" "$BASE_IMAGE_NAME"
fi

BUILD_FLAGS=()
while getopts 'r' opt; do
  case $opt in
  r) BUILD_FLAGS+=(--no-cache) ;;
  *)
    echo 'Error in command line parsing' >&2
    exit 1
    ;;
  esac
done
shift "$((OPTIND - 1))"

BUILD_FLAGS+=(--build-arg ROS_DISTRO="${ROS_DISTRO}")

if [[ "$OSTYPE" != "darwin"* ]]; then
  USER_ID="$(id -u "${USER}")"
  GROUP_ID="$(id -g "${USER}")"
  BUILD_FLAGS+=(--build-arg UID="${USER_ID}")
  BUILD_FLAGS+=(--build-arg GID="${GROUP_ID}")
fi

BUILD_FLAGS+=(-t "${IMAGE_NAME}")
BUILD_FLAGS+=(--network=host)

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" docker
