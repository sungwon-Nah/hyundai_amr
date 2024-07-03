#!/usr/bin/env bash
__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
__file_path="${__dir}/$(basename "${BASH_SOURCE[0]}")"
__file_name="$(basename "${BASH_SOURCE[0]}")"
__call_dir=$(pwd)

export HYUNDAI_AMR_ROOT=$__dir/../../
export AMR_WS=$HYUNDAI_AMR_ROOT/../../
docker compose -f $HYUNDAI_AMR_ROOT/operations/docker-compose.yml up --force-recreate -d hyundaiamr
docker exec --privileged -w "/amr_ws" -it hyundaiamr /bin/bash 
