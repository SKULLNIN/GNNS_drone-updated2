#!/usr/bin/env bash
# gNNS Drone — Ubuntu 24.04 one-shot orchestrator (ROS 2 Jazzy + Gazebo Harmonic)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/scripts/laptop_ubuntu24/env.sh"

cmd="${1:-help}"
shift || true

case "$cmd" in
  install)
    exec bash "${SCRIPT_DIR}/scripts/laptop_ubuntu24/setup_laptop_ubuntu24.sh" --all "$@"
    ;;
  doctor)   gnns_doctor "$@" ;;
  gazebo)   gnns_gazebo "$@" ;;
  sitl)     gnns_sitl "$@" ;;
  mission)  gnns_mission "$@" ;;
  web)      gnns_web "$@" ;;
  fly)      gnns_fly "$@" ;;
  clean)    gnns_clean "$@" ;;
  status)   gnns_status "$@" ;;
  help|-h|--help)
    gnns_help
    ;;
  *)
    echo "Unknown subcommand: $cmd" >&2
    gnns_help
    exit 2
    ;;
esac
