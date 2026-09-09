#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${repo_root}/ros2_ws/install/setup.bash"
source /root/venvs/lerobot/bin/activate
export HF_HUB_OFFLINE=1
export HF_HOME="${repo_root}/Smolvla_trianing/20260909_102253/hf-cache"

exec python "${repo_root}/tools/smolvla_learm_rollout.py" "$@"
