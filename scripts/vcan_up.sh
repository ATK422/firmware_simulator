#!/usr/bin/env bash
# Creates the virtual CAN interfaces an emulated robot's buses are bridged to, ready for CAN FD:
#
#   scripts/vcan_up.sh                 # vcan0 (Talos internal bus) and vcan1 (external bus)
#   scripts/vcan_up.sh vcan0 vcan1 vcan2
#   scripts/vcan_up.sh --down          # removes them again
#
# then, for example:
#
#   firmware_simulator robot robots/talos.toml --can internal=vcan0 --can external=vcan1 --realtime
#   candump -l vcan0
#
# Each interface gets the CAN FD MTU (72 bytes; the internal bus runs CAN FD, and an interface with the classic
# 16 byte MTU rejects FD frames), and a transmit queue long enough for bursts of frames. Needs root (it uses sudo
# when not run as root) and the vcan kernel module, which most desktop kernels ship but many containers and cloud
# VMs lack.
set -euo pipefail

down=0
ifaces=()
for arg in "$@"; do
    case "$arg" in
        --down) down=1 ;;
        -h | --help)
            sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        -*)
            echo "unknown option $arg" >&2
            exit 2
            ;;
        *) ifaces+=("$arg") ;;
    esac
done
if [ "${#ifaces[@]}" -eq 0 ]; then
    ifaces=(vcan0 vcan1)
fi

sudo=()
if [ "$(id -u)" -ne 0 ]; then
    sudo=(sudo)
fi

exists() {
    ip link show "$1" >/dev/null 2>&1
}

if [ "$down" -eq 1 ]; then
    for i in "${ifaces[@]}"; do
        if exists "$i"; then
            "${sudo[@]}" ip link delete dev "$i"
            echo "$i removed"
        fi
    done
    exit 0
fi

if ! "${sudo[@]}" modprobe vcan; then
    echo "could not load the vcan module: this kernel has no virtual CAN support (common in containers and" >&2
    echo "cloud VMs). Use the emulator's in-process bus, or --cannelloni to reach a machine that has vcan." >&2
    exit 1
fi

for i in "${ifaces[@]}"; do
    if ! exists "$i"; then
        "${sudo[@]}" ip link add dev "$i" type vcan
    fi
    # The MTU can only change while the interface is down; leave one that is already right (and maybe in use) alone
    if ! ip link show "$i" | grep -q 'mtu 72 '; then
        "${sudo[@]}" ip link set dev "$i" down
        "${sudo[@]}" ip link set dev "$i" mtu 72
    fi
    "${sudo[@]}" ip link set dev "$i" txqueuelen 1000
    "${sudo[@]}" ip link set dev "$i" up
    echo "$i up (CAN FD MTU 72)"
done
