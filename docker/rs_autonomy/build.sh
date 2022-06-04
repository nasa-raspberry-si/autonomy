#!/bin/bash

# Currently Supported Inputs (OWIMG, DISTRO, IMAGE_VERSION, DOCKERHUB_ORG): 
## e.g., (aisys/raspberrysi-oceanworld:v8, melodic, v1, aisys, ow8-rosnodes),
## e.g., (aisys/raspberrysi-oceanworld:v9, noetic, v2, aisys, ow9)

OWIMG=$1 # oceanwater image: aisys/raspberrysi-oceanworld:v8, aisys/raspberrysi-oceanworld:v9 
DISTRO=$2
IMAGE_VERSION=$3 
DOCKERHUB_ORG=$4
CODE_BRANCH=$5 # the value indicates which branch of github.com/nasa-raspberry-si/autonomy is used

echo -e "Inputs:\n\tOWIMG:${OWIMG}\n\tDISTRO:${DISTRO}\n\tIMAGE_VERSION:${IMAGE_VERSION}\n\tDOCKERHUB_ORG:${DOCKERHUB_ORG}\n\tCODE_BRANCH:${CODE_BRANCH}\n"

IMAGE_NAME=raspberrysi-autonomy

BUILD_CMD=(docker build --no-cache --build-arg OWIMG="${OWIMG}" --build-arg DISTRO="${DISTRO}" --build-arg CODE_BRANCH=${CODE_BRANCH} -t ${DOCKERHUB_ORG}/${IMAGE_NAME}:${IMAGE_VERSION} -f Dockerfile .)
echo -e "Docker Build Command:\n\t${BUILD_CMD[@]}"

"${BUILD_CMD[@]}"

