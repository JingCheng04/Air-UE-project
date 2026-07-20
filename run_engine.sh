#! /bin/bash

set -e

PX4_DIR="/home/JingCheng/PX4/PX4-Autopilot"
PX4_LOG="/tmp/px4_sitl_none_iris.log"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"

if ! command -v pgrep >/dev/null 2>&1; then
    echo "pgrep not found"
    exit 1
fi

if [ ! -d "$PX4_DIR" ]; then
    echo "PX4 directory not found: $PX4_DIR"
    exit 1
fi

# Always restart PX4 SITL cleanly. Unreal may exit while leaving PX4 alive,
# and reusing that stale process tends to keep broken simulator/MAVLink state.
if pgrep -f "$PX4_BIN" >/dev/null 2>&1; then
    echo "Stopping stale PX4 SITL..."
    pkill -f "$PX4_BIN" || true
    for _ in $(seq 1 20); do
        if ! pgrep -f "$PX4_BIN" >/dev/null 2>&1; then
            break
        fi
        sleep 1
    done
fi

if pgrep -f "$PX4_BIN" >/dev/null 2>&1; then
    echo "Failed to stop stale PX4 SITL"
    exit 1
fi

: > "$PX4_LOG"
nohup bash -lc "pushd \"$PX4_DIR\" >/dev/null && make px4_sitl_default none_iris" >"$PX4_LOG" 2>&1 &

echo "Starting PX4 SITL, waiting for simulator TCP port 4560..."
for _ in $(seq 1 120); do
    if grep -Eq "Waiting for simulator to .* connection on TCP port 4560" "$PX4_LOG"; then
        break
    fi
    sleep 1
done

if ! grep -Eq "Waiting for simulator to .* connection on TCP port 4560" "$PX4_LOG"; then
    echo "PX4 did not become ready in time. Check $PX4_LOG"
    exit 1
fi

echo "PX4 log: $PX4_LOG"

$HOME/UnrealEngine/Engine/Binaries/Linux/UnrealEditor $HOME/Air-UE-project/Unreal/Environments/Blocks/Blocks.uproject
