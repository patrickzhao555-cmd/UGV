#!/usr/bin/env bash
set -euo pipefail

display="${1:-${VNC_DISPLAY:-:1}}"
geometry="${VNC_GEOMETRY:-1920x1080}"
depth="${VNC_DEPTH:-24}"

if ! command -v vncserver >/dev/null 2>&1; then
  echo "vncserver was not found. Install TigerVNC on the Nano first:"
  echo "  sudo apt update && sudo apt install -y tigervnc-standalone-server tigervnc-common"
  exit 1
fi

if vncserver -list | grep -Eq "(^|[[:space:]])${display}([[:space:]]|$)"; then
  echo "TigerVNC ${display} is already running."
else
  vncserver -localhost no "${display}" -geometry "${geometry}" -depth "${depth}"
fi

ip_addr="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [[ -n "${ip_addr}" ]]; then
  echo "Connect from Windows TigerVNC Viewer to ${ip_addr}${display}"
  echo "Equivalent TCP port: ${ip_addr}:$((5900 + ${display#:}))"
else
  echo "Connect from Windows TigerVNC Viewer to <nano-ip>${display}"
fi
