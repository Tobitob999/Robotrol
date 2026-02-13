#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root"
  exit 1
fi

APP_DIR="/opt/edge"
ETC_DIR="/etc/edge"
REQ_FILE="${APP_DIR}/requirements.txt"

apt-get update
apt-get install -y python3-venv python3-pip python3-dev v4l-utils curl nftables

mkdir -p "${APP_DIR}" "${ETC_DIR}"

if [[ ! -d "${APP_DIR}/venv" ]]; then
  python3 -m venv "${APP_DIR}/venv"
fi

"${APP_DIR}/venv/bin/pip" install --upgrade pip
"${APP_DIR}/venv/bin/pip" install -r "${REQ_FILE}"

if [[ ! -f "${ETC_DIR}/edge.yaml" ]]; then
  cp "${APP_DIR}/config/edge.yaml" "${ETC_DIR}/edge.yaml"
fi

cp "${APP_DIR}/systemd/edge-core.service" /etc/systemd/system/edge-core.service
systemctl daemon-reload
systemctl enable edge-core.service

echo "Edge install complete"
