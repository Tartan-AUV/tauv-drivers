#!/usr/bin/env bash
# Installs a udev rule that sets the FTDI latency timer to 1 ms for the Xsens
# MTi USB converter (vendor 2639). The default 16 ms makes the converter hand
# packets over in bursts, and the driver's live-data callback drops all but the
# newest one in each burst (~10% of samples lost at 100 Hz).
# Run on the host (not in the container): sudo ./install_udev_rules.sh
set -euo pipefail

RULE_FILE=/etc/udev/rules.d/99-xsens-latency.rules

if [[ $EUID -ne 0 ]]; then
    echo "must be run as root: sudo $0" >&2
    exit 1
fi

cat > "$RULE_FILE" <<'RULE'
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTRS{idVendor}=="2639", ATTR{latency_timer}="1"
RULE
echo "wrote $RULE_FILE"

udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=usb-serial

sleep 1
for dev in /sys/bus/usb-serial/devices/ttyUSB*; do
    [[ -e "$dev/latency_timer" ]] || continue
    echo "$(basename "$dev"): latency_timer=$(cat "$dev/latency_timer")"
done
