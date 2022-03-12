#!/bin/bash

# Currently Supported Inputs
# (DISTRO, IMAGE_VERSION): (melodic, r8), (noetic, r9)
DISTRO=$1
IMAGE_VERSION=$2

echo -e "Inputs:\n\tDISTRO:${DISTRO}\n\tIMAGE_VERSION:${IMAGE_VERSION}\n"

DOCKERHUB_ORG=raspberrysi
IMAGE_NAME=oceanworld
DIRECTORY="versions/${IMAGE_VERSION}"


BUILD_CMD=(docker build --no-cache --build-arg DISTRO="${DISTRO}" --build-arg DIRECTORY="${DIRECTORY}" -t ${DOCKERHUB_ORG}/${IMAGE_NAME}:${IMAGE_VERSION} -f ${DIRECTORY}/Dockerfile .)
echo -e "Docker Build Command:\n\t${BUILD_CMD[@]}"

"${BUILD_CMD[@]}"

