#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
windows_host="${LEARM_WINDOWS_HOST:-$(ip route show default | awk '/default/ {print $3; exit}')}"

if [[ -z "${windows_host}" ]]; then
  echo "cannot determine Windows host; set LEARM_WINDOWS_HOST" >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "${repo_root}/ros2_ws/install/setup.bash"

exec ros2 launch learm_vla_bridge tcp_inference.launch.py \
  windows_host:="${windows_host}" "$@"
