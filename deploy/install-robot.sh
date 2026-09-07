#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
[[ "$PWD" == /home/wheeltec/formation_ws ]] || { echo 'Clone into /home/wheeltec/formation_ws first.' >&2; exit 1; }
case "${1:-}" in ugv0|ugv1|ugv2) ;; *) echo 'Usage: bash deploy/install-robot.sh ugv0|ugv1|ugv2' >&2; exit 1;; esac
mkdir -p /home/wheeltec/.config/formation /home/wheeltec/.config/systemd/user .local
if [[ ! -e /home/wheeltec/.config/formation/robot.env ]]; then
    install -m 644 "deploy/robots/$1.env" /home/wheeltec/.config/formation/robot.env
fi
git config pull.ff only
# This LAN URL must bypass any shell-level HTTP proxy.
git config http.proxy ''
bash scripts/update.sh
if [[ ! -f devel/setup.bash ]]; then
    echo 'Workspace has not been built; stop local ROS nodes and retry.' >&2
    exit 1
fi
source_line='source /home/wheeltec/formation_ws/scripts/env.sh'
if ! grep -Fxq "$source_line" /home/wheeltec/.bashrc; then
    cp -p /home/wheeltec/.bashrc ".local/bashrc.before-formation.$(date +%Y%m%d%H%M%S)"
    printf '\n%s\n' "$source_line" >> /home/wheeltec/.bashrc
fi
install -m 644 deploy/formation-update.service deploy/formation-update.timer /home/wheeltec/.config/systemd/user/
loginctl enable-linger wheeltec
systemctl --user daemon-reload
systemctl --user enable --now formation-update.timer
echo "Configured $1; open a new terminal or source scripts/env.sh."
