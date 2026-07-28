#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ABOT_ROOT="${ABOT_ROOT:-${HOME}/ABot-Claw}"
ROS_CONTAINER="${ROS_CONTAINER:-abot-piper-noetic}"

if [[ ! -d "${REPO_ROOT}/piper-on-bunker" ]]; then
  echo "Repository root not found: ${REPO_ROOT}" >&2
  exit 1
fi

if [[ ! -d "${ABOT_ROOT}" ]]; then
  echo "ABot-Claw root not found: ${ABOT_ROOT}" >&2
  exit 1
fi

if [[ $# -eq 0 ]]; then
  echo "Usage: $0 <command ...>" >&2
  exit 2
fi

if [[ -n "${ROS_NOETIC_IMAGE:-}" ]]; then
  IMAGE="${ROS_NOETIC_IMAGE}"
else
  IMAGE="$(docker inspect "${ROS_CONTAINER}" --format '{{.Config.Image}}' 2>/dev/null || true)"
fi

if [[ -z "${IMAGE}" ]]; then
  echo "Could not determine ROS Noetic image. Set ROS_NOETIC_IMAGE or start ${ROS_CONTAINER}." >&2
  exit 1
fi

CMD_STRING="$(printf '%q ' "$@")"
DOCKER_RUN_ARGS=(--rm -i --privileged)

if [[ -t 0 && -t 1 ]]; then
  DOCKER_RUN_ARGS+=(-t)
fi

echo "Using image: ${IMAGE}"
echo "Mounting repo: ${REPO_ROOT} -> /root/piper-pipeline-testbed"
echo "Mounting ABot-Claw: ${ABOT_ROOT} -> /root/ABot-Claw"
echo "Executing: ${CMD_STRING}"

exec docker run "${DOCKER_RUN_ARGS[@]}" \
  --network host \
  -v "${ABOT_ROOT}:/root/ABot-Claw" \
  -v "${REPO_ROOT}:/root/piper-pipeline-testbed" \
  -w /root/piper-pipeline-testbed \
  "${IMAGE}" \
  bash -lc "source /opt/ros/noetic/setup.bash && if [[ -r /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash ]]; then source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash; fi && ${CMD_STRING}"
