#!/usr/bin/bash

docker build -t sensorfusion .

docker run -t sensorfusion -e DISPLAY=$DISPLAY 