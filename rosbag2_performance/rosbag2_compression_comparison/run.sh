#!/usr/bin/env bash

# usage: run.sh <your input bag directory or filename>
set -e 

rm -rf out
mkdir out

for name in output_config/*.yaml; do
	echo "converting with $name..." 1>&2
	ros2 bag convert -i $1 -o $name 1>&2
done

echo "name, size_bytes"
find out -type f ! -name metadata.yaml -exec stat --printf="%N, %s\n" {} \;
