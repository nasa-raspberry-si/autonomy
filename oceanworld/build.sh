#!/bin/bash

DISTRO=melodic
DIRECTORY=oceanworld
DOCKERHUB_ORG=raspberrysi
IMAGE_NAME=oceanworld
IMAGE_VERSION=nvidia

docker build --no-cache --build-arg DISTRO=${DISTRO} --build-arg DIRECTORY=${DIRECTORY} -t ${DOCKERHUB_ORG}/${IMAGE_NAME}:${IMAGE_VERSION} .
