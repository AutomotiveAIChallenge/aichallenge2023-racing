#!/bin/bash
rocker --device /dev/dri --x11 --user --net host --privileged --volume output:/output -- aichallenge-eval
