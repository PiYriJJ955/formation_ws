#!/usr/bin/env bash
set -e
[[ "${1:-}" =~ ^ugv(0|[1-9][0-9]*)$ ]] || { echo 'Usage: install-robot.sh ugvN [workspace [repository [robot_ip [master_ip]]]]' >&2; exit 1; }
robot_id=${1#ugv}
workspace=${2:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
repository=${3:-http://192.168.0.117:8000/formation.git}
robot_ip=${4:-}
master_ip=${5:-192.168.0.106}
for address in "$master_ip" "$robot_ip"; do
    [[ -z "$address" ]] && continue
    [[ "$address" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]] || { echo 'Expected IPv4 address.' >&2; exit 1; }
    IFS=. read -r -a octets <<< "$address"
    for octet in "${octets[@]}"; do
        ((10#$octet <= 255)) || { echo 'Invalid IPv4 address.' >&2; exit 1; }
    done
done
config_root=$HOME/.config
mkdir -p "$config_root/formation"
exec 8>"$config_root/formation/install.lock"
flock -n 8 || { echo 'Another installation is running; retry later.' >&2; exit 75; }
clone_dir=''
trap '[[ -z "$clone_dir" ]] || rm -rf -- "$clone_dir"' EXIT
if [[ ! -e "$workspace" ]]; then
    mkdir -p "$(dirname "$workspace")"
    clone_dir=$(mktemp -d "$(dirname "$workspace")/.formation-clone.XXXXXX")
    git -c http.proxy= -c http.lowSpeedLimit=1 -c http.lowSpeedTime=20 clone -- "$repository" "$clone_dir"
    mv -T -n -- "$clone_dir" "$workspace"
    [[ ! -e "$clone_dir" ]] || { echo 'Workspace appeared during clone; existing files preserved.' >&2; exit 1; }
    clone_dir=''
fi
cd "$workspace"
workspace=$PWD
[[ -e .git && -f scripts/update.sh && -f scripts/build.sh && -d src/turn_on_wheeltec_robot ]] || { echo 'Target is not a formation Git workspace; existing files preserved.' >&2; exit 1; }
mkdir -p .local
git remote set-url origin "$repository"
git config pull.ff only
# This LAN URL must bypass any shell-level HTTP proxy.
git config http.proxy ''
bash scripts/update.sh --discard-local-changes
revision=$(git rev-parse HEAD)
if [[ ! -f devel/setup.bash || ! -f .local/built-revision ]] || \
   [[ "$(<.local/built-revision)" != "$revision" || "$(git rev-parse origin/master)" != "$revision" ]]; then
    echo 'Update/build pending; stop local ROS nodes and retry.' >&2
    exit 75
fi
if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
    echo 'Local changes found; existing files preserved.' >&2
    exit 1
fi
config_file="$config_root/formation/robot.env"
if [[ ! -e "$config_file" ]]; then
    template="deploy/robots/ugv${robot_id}.env"
    if [[ -f "$template" ]]; then
        install -m 644 "$template" "$config_file"
    else
        printf 'export UGV_ID=%s\nexport ROS_MASTER_URI=http://%s:11311\nexport ROS_IP=%s\nunset ROS_HOSTNAME\nexport CAR_MODE=mini_4wd\n# Set the actual UWB serial device before running localization.\nexport UWB_PORT=\n' \
            "$robot_id" "$master_ip" "$robot_ip" > "$config_file"
    fi
    # These arguments are IPv4 addresses validated by the console.
    [[ -z "$robot_ip" ]] || sed -i "s/^export ROS_IP=.*/export ROS_IP=$robot_ip/" "$config_file"
    sed -i "s|^export ROS_MASTER_URI=.*|export ROS_MASTER_URI=http://$master_ip:11311|" "$config_file"
fi
printf -v source_line 'source %q' "$workspace/scripts/env.sh"
if ! grep -Fxq "$source_line" "$HOME/.bashrc"; then
    [[ ! -f "$HOME/.bashrc" ]] || cp -p "$HOME/.bashrc" ".local/bashrc.before-formation.$(date +%Y%m%d%H%M%S)"
    printf '\n%s\n' "$source_line" >> "$HOME/.bashrc"
fi
mkdir -p "$config_root/systemd/user"
# Quote paths for systemd, including literal percent signs and backslashes.
service_workspace=${workspace//\\/\\\\}
service_workspace=${service_workspace//\"/\\\"}
service_workspace=${service_workspace//%/%%}
service_workspace=${service_workspace//\$/\$\$}
cat > "$config_root/systemd/user/formation-update.service" <<EOF
[Unit]
Description=Fetch and build the formation workspace while ROS is stopped
Wants=network-online.target
After=network-online.target
[Service]
Type=oneshot
ExecStart=/bin/bash "$service_workspace/scripts/update.sh"
TimeoutStartSec=20min
EOF
install -m 644 deploy/formation-update.timer "$config_root/systemd/user/"
loginctl enable-linger "$(id -un)"
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}
systemctl --user daemon-reload
systemctl --user enable --now formation-update.timer
echo "Configured $1; workspace ready: $revision"
