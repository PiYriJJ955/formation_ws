#!/usr/bin/env bash
# Source this file to select this workspace and the current robot's settings.
export FORMATION_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f /home/wheeltec/.config/formation/robot.env ]]; then
    source /home/wheeltec/.config/formation/robot.env
fi
source "$FORMATION_WS/devel/setup.bash"
