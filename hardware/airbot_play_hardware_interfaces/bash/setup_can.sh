#!/usr/bin/env bash
set -e

CAN_IF="can0"
BITRATE_FLAG="-s8"   # -s8 = 1 Mbps

CAN_DEV="$(find /dev/serial/by-id -name '*DISCOVER_Robotics*USB_to_CAN*' | head -n 1)"

if [ -z "$CAN_DEV" ]; then
  echo "DISCOVER Robotics USB-to-CAN adapter not found."
  echo "Available serial devices:"
  ls -l /dev/serial/by-id/ 2>/dev/null || true
  exit 1
fi

echo "Using CAN adapter: $CAN_DEV"

sudo modprobe slcan || true
sudo pkill slcand || true
sudo ip link set "$CAN_IF" down 2>/dev/null || true

sudo slcand -o -c "$BITRATE_FLAG" "$CAN_DEV" "$CAN_IF"
sleep 0.5
sudo ip link set "$CAN_IF" up

echo "$CAN_IF is ready:"
ip -details link show "$CAN_IF"
