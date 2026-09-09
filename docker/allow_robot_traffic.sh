#!/usr/bin/env bash
set -Eeuo pipefail

iface="$1"
if ! command -v ufw >/dev/null 2>&1 || ! systemctl is-active --quiet ufw; then
    exit 0
fi
sudo -v
if sudo ufw status | grep -Fq "on ${iface}"; then
    exit 0
fi
sudo ufw allow in on "$iface"
