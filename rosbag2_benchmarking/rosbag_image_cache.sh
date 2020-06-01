#!/bin/bash 
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

TMP_DIR=$SCRIPTPATH/tmp
mkdir -p $TMP_DIR

rm -rf $TMP_DIR && ros2 bag record /image --max-cache-size $1 -o $TMP_DIR