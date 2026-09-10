#!/usr/bin/env bash
set -e
# The console may run its uploaded updater against an older vehicle checkout.
cd "${FORMATION_UPDATE_WORKSPACE:-$(dirname "${BASH_SOURCE[0]}")/..}"
discard_local_changes=false
if [[ "${1:-}" == '--discard-local-changes' ]]; then
    discard_local_changes=true
elif [[ -n "${1:-}" ]]; then
    echo 'Usage: scripts/update.sh [--discard-local-changes]' >&2
    exit 2
fi
mkdir -p .local
exec 9>.local/update.lock
flock -n 9 || exit 0
exec > >(tee .local/update.log) 2>&1
git -c http.lowSpeedLimit=1 -c http.lowSpeedTime=20 fetch origin master
# Defer checkout/build while any local ROS launch or controller is running.
if pgrep -f '(^|/)(roslaunch|roscore|rosmaster)( |$)|wheeltec_robot_node|displacement_follower.py|robust_uwb_localizer.py|nlink_localizer.py|/lib/nlink_parser/' >/dev/null; then
    echo 'Fetched; workspace update deferred while ROS is running.'
    exit 0
fi
[[ "$(git symbolic-ref --short HEAD)" == master ]] || { echo 'Expected master branch.' >&2; exit 1; }
git merge-base --is-ancestor HEAD origin/master || { echo 'Local commits are not on the server; update stopped.' >&2; exit 1; }
if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
    if [[ "$discard_local_changes" == true ]]; then
        git reset --hard HEAD
        [[ -z "$(git status --porcelain --untracked-files=normal)" ]] || {
            echo 'Untracked files found; remove or move them before updating.' >&2
            exit 1
        }
    else
        echo 'Local changes found; commit or move them before updating.' >&2
        exit 1
    fi
fi
git merge --ff-only origin/master
revision=$(git rev-parse HEAD)
if [[ ! -f .local/built-revision ]] || [[ "$(<.local/built-revision)" != "$revision" ]]; then
    bash scripts/build.sh
    echo "$revision" > .local/built-revision
fi
echo "Workspace ready: $revision"
