#!/usr/bin/env bash
set -euo pipefail
# Called by build.sh or the GUI after sourcing the system ROS distribution.
[[ -z "${1:-}" || "${1:-}" == --sudo-stdin ]] || { echo "Unknown argument" >&2; exit 2; }
version=${ROS_PYTHON_VERSION:-}
[[ "$version" == 2 || "$version" == 3 ]] || {
    echo 'Source /opt/ros/melodic/setup.bash or /opt/ros/noetic/setup.bash first.' >&2
    exit 1
}
python_bin=$(command -v "python$version")
if "$python_bin" -c 'import numpy; from scipy.optimize import minimize' >/dev/null 2>&1; then
    exit 0
fi
suffix=''
[[ "$version" != 3 ]] || suffix=3
packages=("python${suffix}-numpy" "python${suffix}-scipy")
apt_command=(apt-get)
if ((EUID != 0)); then
    if ! sudo -n true 2>/dev/null; then
        if [[ "${1:-}" == --sudo-stdin ]]; then
            sudo -S -p '' -v
        else
            echo "MPC dependencies missing. Run: sudo apt-get install ${packages[*]}" >&2
            exit 1
        fi
    fi
    apt_command=(sudo -n apt-get)
fi
"${apt_command[@]}" update
"${apt_command[@]}" install -y "${packages[@]}"
"$python_bin" -c 'import numpy; from scipy.optimize import minimize'
