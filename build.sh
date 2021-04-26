#!/usr/bin/env bash

#MULTISTAGE_TARGET="user"

REBUILD=0
while getopts 'r' opt; do
    case $opt in
        r) REBUILD=1 ;;
        *) echo 'Error in command line parsing' >&2
           exit 1
    esac
done
shift "$(( OPTIND - 1 ))"

NAME=$(echo "${PWD##*/}" | tr _ -)
TAG="latest"

BUILD_FLAGS=(--target "${MULTISTAGE_TARGET}")
BUILD_FLAGS+=(-t "${NAME}:${TAG}")

if [ "$REBUILD" -eq 1 ]; then
    BUILD_FLAGS+=(--no-cache)
fi

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}"  .