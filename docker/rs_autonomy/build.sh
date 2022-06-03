#!/bin/bash

# Currently Supported Inputs (OWIMG, DISTRO, IMAGE_VERSION, DOCKERHUB_ORG): 
## e.g., (raspberrysi/oceanworld:r8, melodic, ow8-rosnodes, raspberrysi),
## e.g., (raspberrysi/oceanworld:r9, noetic, ow9, raspberrysi)

OWIMG=$1 # oceanwater image: raspberrysi/oceanworld:r8, raspberrysi/oceanworld:r9 
DISTRO=$2
IMAGE_VERSION=$3 # the value indicates which branch of github.com/nasa-raspberry-si/autonomy is used
DOCKERHUB_ORG=$4

echo -e "Inputs:\n\tOWIMG:${OWIMG}\n\tDISTRO:${DISTRO}\n\tIMAGE_VERSION:${IMAGE_VERSION}\n\tDOCKERHUB_ORG:${DOCKERHUB_ORG}\n"

IMAGE_NAME=rs_autonomy

BUILD_CMD=(docker build --no-cache --build-arg OWIMG="${OWIMG}" --build-arg DISTRO="${DISTRO}" --build-arg IMAGE_VERSION=${IMAGE_VERSION} -t ${DOCKERHUB_ORG}/${IMAGE_NAME}:${IMAGE_VERSION} -f Dockerfile .)
echo -e "Docker Build Command:\n\t${BUILD_CMD[@]}"

"${BUILD_CMD[@]}"

