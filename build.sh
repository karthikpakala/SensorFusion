#!/usr/bin/bash

docker build -t sensorfusion .

#docker run -it --rm -e DISPLAY=$DISPLAY -v /ssd/Projects/Data:/Data sensorfusion