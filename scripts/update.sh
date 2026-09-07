#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p .local
exec 9>.local/update.lock
flock -n 9 || exit 0
git -c http.lowSpeedLimit=1 -c http.lowSpeedTime=20 fetch origin master
# Defer checkout/build while any local ROS launch or controller is running.
if pgrep -f '(^|/)(roslaunch|roscore|rosmaster)( |$)|wheeltec_robot_node|displacement_follower.py|robust_uwb_localizer.py|/lib/nlink_parser/' >/dev/null; then
    echo 'Fetched; workspace update deferred while ROS is running.'
    exit 0
fi
if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
    echo 'Local changes found; commit or move them before updating.' >&2
    exit 1
fi
[[ "$(git symbolic-ref --short HEAD)" == master ]] || { echo 'Expected master branch.' >&2; exit 1; }
git merge --ff-only origin/master
revision=$(git rev-parse HEAD)
if [[ ! -f .local/built-revision ]] || [[ "$(<.local/built-revision)" != "$revision" ]]; then
    bash scripts/build.sh
    echo "$revision" > .local/built-revision
fi
echo "Workspace ready: $revision"
