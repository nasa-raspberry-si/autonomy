#!/bin/bash

# Currently Supported Inputs
# (DISTRO, IMAGE_VERSION, DOCKERHUB_ORG): e.g., (melodic, v8, aisys), (noetic, v9, aisys)
DISTRO=$1
IMAGE_VERSION=$2
DOCKERHUB_ORG=$3

echo -e "Inputs:\n\tDISTRO:${DISTRO}\n\tIMAGE_VERSION:${IMAGE_VERSION}\n\tDOCKERHUB_ORG:${DOCKERHUB_ORG}\n"

IMAGE_NAME=raspberrysi-oceanworld
DIRECTORY="versions/${IMAGE_VERSION}"


BUILD_CMD=(docker build --no-cache --build-arg DISTRO="${DISTRO}" --build-arg DIRECTORY="${DIRECTORY}" -t ${DOCKERHUB_ORG}/${IMAGE_NAME}:${IMAGE_VERSION} -f ${DIRECTORY}/Dockerfile .)
echo -e "Docker Build Command:\n\t${BUILD_CMD[@]}"

"${BUILD_CMD[@]}"

