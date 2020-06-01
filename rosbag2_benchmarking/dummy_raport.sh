#!/usr/bin/env bash
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

TMP_DIR=$SCRIPTPATH/tmp

if [ ! -f "$SCRIPTPATH/tmp/metadata.yaml" ]; then
    echo Stop rosbag record first!
    exit 1
fi


COUNT=`cat $SCRIPTPATH/tmp/count`
SAVED=$(cat $TMP_DIR/metadata.yaml | sed -n 's/message_count: \([0-9]*\)/\1/p' | head -1)

echo "Messages generated: $COUNT, saved in rosbag: $SAVED"