#!/usr/bin/env bash
set -euo pipefail

display="${VNC_DISPLAY:-:1}"
geometry="${VNC_GEOMETRY:-1920x1080}"
depth="${VNC_DEPTH:-24}"

if ! command -v vncserver >/dev/null 2>&1; then
  echo "vncserver was not found. Install TigerVNC first:"
  echo "  sudo apt update && sudo apt install -y tigervnc-standalone-server tigervnc-common"
  exit 1
fi

vnc_bin="$(command -v vncserver)"
service_dir="${HOME}/.config/systemd/user"
service_path="${service_dir}/tigervnc-ugv.service"
mkdir -p "${service_dir}"

cat > "${service_path}" <<EOF
[Unit]
Description=UGV TigerVNC desktop ${display}
After=network-online.target

[Service]
Type=forking
ExecStartPre=-${vnc_bin} -kill ${display}
ExecStart=${vnc_bin} -localhost no ${display} -geometry ${geometry} -depth ${depth}
ExecStop=${vnc_bin} -kill ${display}
Restart=on-failure
RestartSec=3

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable --now tigervnc-ugv.service

if command -v loginctl >/dev/null 2>&1; then
  loginctl enable-linger "${USER}" >/dev/null 2>&1 || true
fi

echo "Installed and started ${service_path}"
echo "Status: systemctl --user status tigervnc-ugv.service"
ip_addr="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [[ -n "${ip_addr}" ]]; then
  echo "Connect from Windows TigerVNC Viewer to ${ip_addr}${display}"
fi
