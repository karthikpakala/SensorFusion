#!/usr/bin/bash

docker build -t sensorfusion .

docker run -it --rm -e DISPLAY=$DISPLAY -v /Users/karthikpakala/Projects/data:/Data sensorfusion