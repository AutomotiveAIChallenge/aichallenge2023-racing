#!/bin/bash
mkdir -p output
rocker --device /dev/dri --nvidia --x11 --user --net host --privileged --volume ../aichallenge:/aichallenge -- aichallenge-train
