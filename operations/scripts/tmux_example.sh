#!/usr/bin/env bash
__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
__file_path="${__dir}/$(basename "${BASH_SOURCE[0]}")"
__file_name="$(basename "${BASH_SOURCE[0]}")"
__call_dir=$(pwd)

export HYUNDAI_AMR_ROOT=$__dir/../../

network_monitor.sh &

docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES=`tput lines` \
        -w "/workspace" -it hyundaiamr /bin/tmuxp load /workspace/operations/config/tmux_example.yaml

wait