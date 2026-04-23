#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/ardupilot"
py Tools/autotest/sim_vehicle.py -v ArduCopter
