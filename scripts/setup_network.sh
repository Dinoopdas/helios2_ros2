#!/bin/bash
# setup_network.sh
# One-shot script to configure the host's network for the Helios2 camera.
# Idempotent — safe to run multiple times.
#
# Usage:
#   ./setup_network.sh [ETHERNET_INTERFACE] [HOST_IP] [CIDR]
#
# Examples:
#   ./setup_network.sh                        # uses defaults
#   ./setup_network.sh enp0s31f6              # explicit interface
#   ./setup_network.sh eth0 192.168.100.5 24  # full custom

set -euo pipefail

# --- Defaults ---
IFACE="${1:-enp0s31f6}"
HOST_IP="${2:-192.168.100.1}"
CIDR="${3:-24}"
CONN_NAME="helios2-cam"

echo "==> Configuring $IFACE with $HOST_IP/$CIDR"

# --- Pre-flight checks ---
if [[ $EUID -ne 0 ]]; then
   echo "This script needs root. Re-run with sudo."
   exit 1
fi

if ! ip link show "$IFACE" &>/dev/null; then
    echo "ERROR: interface $IFACE not found."
    echo "Available interfaces:"
    ip -br link show
    exit 1
fi

# --- Remove old conflicting NM profiles ---
echo "==> Removing conflicting NetworkManager profiles..."
nmcli connection delete "Wired connection 1" 2>/dev/null || true
nmcli connection delete "$CONN_NAME" 2>/dev/null || true

# --- Create the camera profile ---
echo "==> Creating NetworkManager profile '$CONN_NAME'..."
nmcli connection add type ethernet ifname "$IFACE" con-name "$CONN_NAME" \
    ipv4.method manual \
    ipv4.addresses "$HOST_IP/$CIDR" \
    ipv4.never-default yes \
    ipv4.dns "" \
    autoconnect yes

# --- Activate ---
echo "==> Activating profile..."
nmcli connection up "$CONN_NAME"

# --- Network buffer tuning (idempotent) ---
echo "==> Tuning kernel UDP buffers..."
sysctl -w net.core.rmem_max=33554432 >/dev/null
sysctl -w net.core.rmem_default=33554432 >/dev/null
sysctl -w net.core.wmem_max=33554432 >/dev/null

# Add to sysctl.conf if not already there
if ! grep -q "net.core.rmem_max=33554432" /etc/sysctl.conf 2>/dev/null; then
    cat <<'EOF' >> /etc/sysctl.conf

# GigE Vision tuning for Helios2 ToF camera (added by helios2_node setup_network.sh)
net.core.rmem_max=33554432
net.core.rmem_default=33554432
net.core.wmem_max=33554432
EOF
fi

# --- Report ---
echo ""
echo "==> Done."
echo ""
ip addr show "$IFACE" | grep -E "inet |state"
echo ""
echo "Next step: power on the camera and run"
echo "  ping -c 3 192.168.100.40"
echo ""
echo "If ping succeeds, launch the driver:"
echo "  ros2 launch helios2_node helios2.launch.py"
