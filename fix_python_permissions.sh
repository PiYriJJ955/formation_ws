#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
find "${workspace_dir}/src" -path "*/scripts/*.py" -type f -print -exec chmod +x {} \;
echo "ROS Python node permissions fixed under ${workspace_dir}/src."
