#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root"
  exit 1
fi

APP_DIR="/opt/agent"
ETC_DIR="/etc/agent"
DATA_DIR="/var/lib/agent"
REQ_FILE="${APP_DIR}/requirements.txt"

apt-get update
apt-get install -y python3-venv python3-pip python3-dev curl stockfish

mkdir -p "${APP_DIR}" "${ETC_DIR}" "${DATA_DIR}"

if [[ ! -d "${APP_DIR}/venv" ]]; then
  python3 -m venv "${APP_DIR}/venv"
fi

"${APP_DIR}/venv/bin/pip" install --upgrade pip
"${APP_DIR}/venv/bin/pip" install -r "${REQ_FILE}"

if [[ ! -f "${ETC_DIR}/agent.yaml" ]]; then
  cp "${APP_DIR}/config/agent.yaml" "${ETC_DIR}/agent.yaml"
fi

cp "${APP_DIR}/systemd/agent-api.service" /etc/systemd/system/agent-api.service
systemctl daemon-reload
systemctl enable agent-api.service

echo "Init DB"
"${APP_DIR}/venv/bin/python" "${APP_DIR}/scripts/init_db.py" --config "${ETC_DIR}/agent.yaml"

echo "Agent install complete"
