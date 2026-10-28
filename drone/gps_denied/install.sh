#!/bin/bash
# Installs the VAMA flight recorder on a Raspberry Pi (Raspberry Pi OS Bookworm or newer).
#
#     cd gps_denied
#     sudo ./install.sh
#
# It copies the recorder to /opt/vama-recorder, makes a Python environment there that also sees the
# Pi's own camera library (picamera2), puts the settings in /etc/vama-recorder.ini (an existing one
# is kept), and starts the recorder at every power on. It does not change the serial port or the
# camera settings of the Pi: see RECORDER.md, part 3.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Run it with sudo: sudo ./install.sh"
    exit 1
fi
HERE="$(cd "$(dirname "$0")" && pwd)"
USER_NAME="${SUDO_USER:-$(logname 2>/dev/null || echo pi)}"
if [ "$USER_NAME" = "root" ]; then
    echo "Run it with sudo from your normal user, not as root."
    exit 1
fi

echo "== The system packages (picamera2 comes with Raspberry Pi OS)"
apt-get install -y python3-picamera2 python3-venv

echo "== /opt/vama-recorder"
mkdir -p /opt/vama-recorder
rm -rf /opt/vama-recorder/vama_recorder
cp -r "$HERE/vama_recorder" /opt/vama-recorder/
if [ ! -d /opt/vama-recorder/venv ]; then
    python3 -m venv --system-site-packages /opt/vama-recorder/venv
fi
/opt/vama-recorder/venv/bin/pip install --upgrade pymavlink
chown -R "$USER_NAME:" /opt/vama-recorder

echo "== /etc/vama-recorder.ini"
if [ -f /etc/vama-recorder.ini ]; then
    echo "kept the settings already there"
else
    cp "$HERE/recorder.ini" /etc/vama-recorder.ini
fi

echo "== The service (starts at every power on)"
sed "s/__USER__/$USER_NAME/" "$HERE/vama-recorder.service" > /etc/systemd/system/vama-recorder.service
usermod -a -G dialout,video "$USER_NAME"
systemctl daemon-reload
systemctl enable vama-recorder
systemctl restart vama-recorder

echo
echo "Installed. The recorder runs as $USER_NAME and saves flights in that user's ~/vama-flights."
echo "Check it on the bench (the service holds the serial port, so stop it first):"
echo "    sudo systemctl stop vama-recorder"
echo "    cd /opt/vama-recorder && venv/bin/python -m vama_recorder --check"
echo "    sudo systemctl start vama-recorder"
echo "Its messages:   journalctl -u vama-recorder -f"
