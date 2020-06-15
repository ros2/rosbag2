#!/usr/bin/env bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

echo "--------------"
echo "This script copies iotop policy to '/usr/share/polkit-1/actions/iotop.policy' so it will always be allowed to with root privileges."
echo ""
echo "File contents: "
echo ""
cat $SCRIPTPATH/iotop.policy
echo ""
echo "--------------"

sudo cp $SCRIPTPATH/iotop.policy /usr/share/polkit-1/actions/iotop.policy