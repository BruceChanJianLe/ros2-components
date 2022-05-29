#!/bin/bash
# This script will start the command centre and lifecycle_demo

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd -P)

usage() {
  cat <<EOF
Usage: $(basename "${BASH_SOURCE[0]}") [-h] [-p]
Script description here.
Available options:
-h, --help               Print this help and exit
-w, --workspacepath      Absolute path to workspace folder, default to $HOME/colcon_ws
EOF
  exit
}

setup_colors() {
  if [[ -t 2 ]] && [[ -z "${NO_COLOR-}" ]] && [[ "${TERM-}" != "dumb" ]]; then
    NOFORMAT='\033[0m' RED='\033[0;31m' GREEN='\033[0;32m' ORANGE='\033[0;33m' BLUE='\033[0;34m' PURPLE='\033[0;35m' CYAN='\033[0;36m' YELLOW='\033[1;33m'
  else
    NOFORMAT='' RED='' GREEN='' ORANGE='' BLUE='' PURPLE='' CYAN='' YELLOW=''
  fi
}

msg() {
  echo >&2 -e "${1-}"
}

die() {
  local msg=$1
  local code=${2-1} # default exit status 1
  msg "$msg"
  exit "$code"
}

parse_params() {
  # default values of variables set from params
  WSPATH="$HOME/colcon_ws"

  while test $# -gt 0; do
    case $1 in
    -h | --help) usage ;;
    -w | --workspacepath)
      shift
      WSPATH=$1
      ;;
    -?*) die "Unknown option: $1" ;;
    *) break ;;
    esac
    shift
  done

  return 0
}

parse_params "$@"
WSPATH=$WSPATH'/install/setup.bash'

tmux has-session -t lifecycle_demo 2>/dev/null

if [ $? != 0 ]; then
    # create a new session and attach
    tmux new-session -s lifecycle_demo -n lifecycle_demo -d

    # Lifecycle container
    tmux send-keys -t lifecycle_demo 'source '$WSPATH C-m
    tmux send-keys -t lifecycle_demo 'tmux select-layout tiled; clear' C-m
    tmux send-keys -t lifecycle_demo 'ros2 run rclcpp_components component_container --ros-args --remap __node:=the_container' C-m

    # Attach lifecycle component talker
    tmux split-window -h -t lifecycle_demo
    tmux send-keys -t lifecycle_demo 'source '$WSPATH C-m
    tmux send-keys -t lifecycle_demo 'tmux select-layout tiled; clear' C-m
    tmux send-keys -t lifecycle_demo 'ros2 component load /the_container ros2-components lifecycle_component::component_talker'

    # configure lifecycle component talker
    tmux split-window -v -t lifecycle_demo
    tmux send-keys -t lifecycle_demo 'source '$WSPATH C-m
    tmux send-keys -t lifecycle_demo 'tmux select-layout tiled; clear' C-m
    tmux send-keys -t lifecycle_demo 'ros2 lifecycle set /lifecycle_component_talker_node configure'

    # activate lifecycle component talker
    tmux split-window -h -t lifecycle_demo
    tmux send-keys -t lifecycle_demo 'source '$WSPATH C-m
    tmux send-keys -t lifecycle_demo 'tmux select-layout tiled; clear' C-m
    tmux send-keys -t lifecycle_demo 'ros2 lifecycle set /lifecycle_component_talker_node activate'
fi

if [ -z "$TMUX" ] && [ ${UID} != 0 ]
then
    tmux attach-session -t lifecycle_demo
else
    tmux switch-client -t lifecycle_demo
fi