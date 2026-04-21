#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULE_SRC="${REPO_ROOT}/scripts/19-footswitch.rules"
RULE_DST="/etc/udev/rules.d/19-footswitch.rules"

if [[ ! -f "${RULE_SRC}" ]]; then
  echo "udev rule not found: ${RULE_SRC}" >&2
  exit 1
fi

sudo cp "${RULE_SRC}" "${RULE_DST}"
sudo chmod 0644 "${RULE_DST}"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed ${RULE_DST}"
echo "Reconnect the footswitch before running the ROS 2 node."
