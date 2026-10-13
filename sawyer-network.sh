#!/usr/bin/env bash
# Red de laboratorio para Sawyer y sensor ATI Net F/T (Debian 13).
# Uso: ./sawyer-network.sh configure | check
set -euo pipefail

IFACE="${IFACE:-enp46s0}"
PROFILE="${PROFILE:-Wired connection 1}"
FT_SENSOR_IP="${FT_SENSOR_IP:-192.168.1.11}"
ROBOT_IP="${ROBOT_IP:-169.254.82.186}"

# Direcciones de esta computadora en la MISMA interfaz Ethernet.
ATI_HOST="${ATI_HOST:-192.168.1.20/24}"
SAWYER_HOST="${SAWYER_HOST:-169.254.82.187/16}"

usage() {
  printf 'Uso: %s {configure|check}\n' "${0##*/}" >&2
  exit 2
}

configure() {
  command -v nmcli >/dev/null || { echo 'Falta nmcli (NetworkManager).' >&2; exit 1; }
  ip link show dev "$IFACE" >/dev/null 2>&1 || { echo "No existe la interfaz $IFACE" >&2; exit 1; }
  nmcli connection show "$PROFILE" >/dev/null || { echo "No existe el perfil: $PROFILE" >&2; exit 1; }

  # Evita DHCP y la puerta de enlace de Ethernet. Wi-Fi conserva Internet.
  sudo nmcli connection modify "$PROFILE" \
    connection.interface-name "$IFACE" \
    connection.autoconnect yes \
    ipv4.method manual \
    ipv4.addresses "$ATI_HOST,$SAWYER_HOST" \
    ipv4.gateway '' \
    ipv4.dns '' \
    ipv4.never-default yes

  sudo nmcli connection up "$PROFILE" ifname "$IFACE"
  printf '\nDirecciones persistentes configuradas en %s:\n' "$IFACE"
  ip -br -4 address show dev "$IFACE"
  printf '\nUsa "%s check" para verificar las rutas y el enlace.\n' "${0##*/}"
}

check() {
  printf 'Direcciones IPv4 en %s:\n' "$IFACE"
  ip -br -4 address show dev "$IFACE"
  printf '\nRuta al sensor ATI (%s):\n' "$FT_SENSOR_IP"
  ip -4 route get "$FT_SENSOR_IP"
  printf '\nRuta al Sawyer (%s):\n' "$ROBOT_IP"
  ip -4 route get "$ROBOT_IP"

  if [[ " $(ip -4 route get "$FT_SENSOR_IP") " != *" dev $IFACE "* ]] || \
     [[ " $(ip -4 route get "$ROBOT_IP") " != *" dev $IFACE "* ]]; then
    echo 'ERROR: alguna ruta no utiliza la interfaz Ethernet.' >&2
    exit 1
  fi

  printf '\nPrueba del sensor ATI:\n'
  ping -I "$IFACE" -c 2 -W 2 "$FT_SENSOR_IP"
  printf '\nPrueba del Sawyer:\n'
  ping -I "$IFACE" -c 2 -W 2 "$ROBOT_IP"
}

case "${1:-}" in
  configure) configure ;;
  check)     check ;;
  *)         usage ;;
esac
