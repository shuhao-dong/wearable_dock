#!/usr/bin/env bash
set -euo pipefail
IDX=${1:-0}
HCI="hci${IDX}"

echo "[scan] chosen adapter: $HCI"

# 1. find the rfkill index that owns this adapte
RF_ID=$(rfkill list | awk -v h="$HCI" '
    /^[0-9]+:/             { id=$1; sub(/:$/,"",id); next }   
    /Bluetooth/ && $0~h    { print id; exit }                 
')

if [[ -n "$RF_ID" ]]; then
    rfkill unblock "$RF_ID"  
fi

# 2. bring interface up (safe if already UP)
hciconfig "$HCI" up || {
    echo "[scan] ERROR: cannot bring $HCI up" >&2
    exit 1
}

# 3. exec the real program (PID stays the same for systemd)
exec /usr/local/bin/scan "$IDX"
