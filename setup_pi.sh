#!/bin/bash
# ATEM USB capture setup on the Pi (run as root).
set -x

# 1. Allow the desktop user to access Blackmagic USB devices.
cat > /etc/udev/rules.d/99-atem.rules <<'RULE'
SUBSYSTEM=="usb", ATTR{idVendor}=="1edb", MODE="0666"
RULE
udevadm control --reload-rules
udevadm trigger

# 2. usbmon + debugfs
modprobe usbmon || true
if [ ! -d /sys/kernel/debug/usb/usbmon ]; then
    mount -t debugfs none /sys/kernel/debug || true
fi

# 3. tcpdump (gives pcap that Wireshark can read directly)
apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq tcpdump

echo "=== RESULT ==="
ls -l /dev/bus/usb/001/
echo "--- usbmon ---"
ls /sys/kernel/debug/usb/usbmon/ 2>/dev/null
echo "--- tcpdump ---"
which tcpdump && tcpdump --version 2>&1 | head -1
