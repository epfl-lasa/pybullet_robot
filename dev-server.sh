#!/usr/bin/env bash
if [ "$(uname -s)" != "Linux" ]; then
  echo "This Docker image is currently only running on Linux. Aborting..."
  exit 1
fi

REBUILD=0
while getopts 'r' opt; do
    case $opt in
        r) REBUILD=1 ;;
        *) echo 'Error in command line parsing' >&2
           exit 1
    esac
done
shift "$(( OPTIND - 1 ))"

IMAGE_NAME=$(echo "${PWD##*/}" | tr _ -)
TAG="latest"
CONTAINER_NAME="$(echo "${PWD##*/}" | tr _ -)"
VOLUME_NAME="$(echo "${PWD##*/}" | tr _ -)_vol"

if [ "$REBUILD" -eq 1 ]; then
  DOCKER_BUILDKIT=1 docker build --no-cache . --tag $IMAGE_NAME
else
  DOCKER_BUILDKIT=1 docker build . --tag $IMAGE_NAME
fi

docker container stop "$CONTAINER_NAME" >/dev/null 2>&1
docker rm --force "$CONTAINER_NAME" >/dev/null 2>&1
docker volume rm "$VOLUME_NAME" >/dev/null 2>&1

docker volume create --driver local --opt type=none --opt device=$PWD --opt o=bind "$VOLUME_NAME"

xhost +
docker run --privileged --net=host -it --rm --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$VOLUME_NAME:/pybullet:rw" \
  --name $CONTAINER_NAME $IMAGE_NAME:$TAG
