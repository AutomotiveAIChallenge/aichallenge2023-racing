#!/bin/bash
rocker  --device /dev/dri --nvidia --x11 --user --net host --privileged --volume output:/output -- aichallenge-eval
