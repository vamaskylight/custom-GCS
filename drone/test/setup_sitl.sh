#!/bin/bash
# Download the ArduCopter simulator programs into the WSL home folder.
# Run it once inside WSL (Ubuntu):  bash setup_sitl.sh
# The tests then start them from ~/vama-sitl.
set -e

VERSIONS="4.6.2 4.7.0"
DEST="$HOME/vama-sitl"

fetch() {
    # fetch <url> <file>: try a few times, the connection is sometimes reset
    curl -fsSL --retry 6 --retry-all-errors --retry-delay 3 -o "$2.part" "$1"
    mv "$2.part" "$2"
}

for version in $VERSIONS; do
    mkdir -p "$DEST/$version"
    if [ ! -x "$DEST/$version/arducopter" ]; then
        echo "Downloading ArduCopter $version simulator"
        fetch "https://firmware.ardupilot.org/Copter/stable-$version/SITL_x86_64_linux_gnu/arducopter" "$DEST/$version/arducopter"
        chmod +x "$DEST/$version/arducopter"
    fi
    if [ ! -s "$DEST/$version/copter.parm" ]; then
        echo "Downloading the default parameters of $version"
        fetch "https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-$version/Tools/autotest/default_params/copter.parm" "$DEST/$version/copter.parm"
    fi
    echo "$version: ready in $DEST/$version"
done
echo "Ready."
