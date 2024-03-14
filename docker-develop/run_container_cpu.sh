#!/bin/bash
mkdir -p output
rocker --device /dev/dri --x11 --user --net host --privileged --volume ../aichallenge:/aichallenge -- aichallenge-202407-develop
