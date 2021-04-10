#!/usr/bin/env bash

NAME=$(echo "${PWD##*/}" | tr _ -)
TAG="latest"

xhost +
docker run --privileged --net=host -it --rm --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" $NAME:$TAG
