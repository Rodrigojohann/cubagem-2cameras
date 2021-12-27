#!/bin/bash
xhost +local:docker
docker build --no-cache -t cubagem-test:1.0 .
docker run --device /dev/dri/:/dev/dri -ti -e DISPLAY=unix$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --privileged cubagem-test:1.0
