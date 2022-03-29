#!/bin/sh
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" julia-image
export containerId=$(docker ps -l -q)
xhost +local:root # for the lazy and reckless