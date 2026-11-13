#!/usr/bin/env bash
# Start the WSLg Gazebo GUI for HomeFly with every required environment
# variable bound to THIS invocation, then auto-focus the camera on homefly.
#
# Why this exists: launching the GUI by hand repeatedly stalled at
#     [GUI] [Dbg] [Gui.cc:356] GUI requesting list of world names...
# because GZ_IP=127.0.0.1 (WSLg multicast discovery workaround) and
# GZ_SIM_RESOURCE_PATH (needed by model://runway) were exported in a different
# terminal or forgotten. Shell exports do NOT cross terminals; binding them as
# command-scoped variables inside this script cannot be forgotten.
#
# Usage:
#     bash tools/run_wsl_gui.sh                                   # airport + auto focus
#     bash tools/run_wsl_gui.sh --world tools/gazebo/homefly_quad.world
#     bash tools/run_wsl_gui.sh --no-focus                       # plain Gazebo GUI
#
# This is terminal 1 ONLY. Terminals 2/3/4 (bridge / FC / GCS) are unchanged;
# see docs/gazebo.md sections 8.7 / 8.8. Terminal 2 still needs its own
# GZ_IP=127.0.0.1.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

WORLD="tools/gazebo/homefly_airport.world"
FOCUS=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --world) WORLD="$2"; shift 2 ;;
        --no-focus) FOCUS=0; shift ;;
        -h|--help)
            echo "Usage: bash tools/run_wsl_gui.sh [--world PATH] [--no-focus]"
            echo "  --world PATH  world SDF (default: tools/gazebo/homefly_airport.world)"
            echo "  --no-focus    do not auto-frame homefly after the GUI comes up"
            echo "  This is terminal 1 only; see docs/gazebo.md 8.7 / 8.8 for 2/3/4."
            exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; exit 1 ;;
    esac
done

# Bound to this process (and its children: gz server, GUI, focus watcher).
export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_SIM_RESOURCE_PATH="$ROOT/tools/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

if ! command -v gz >/dev/null 2>&1; then
    echo "ERROR: 'gz' CLI not found in PATH." >&2
    exit 1
fi
if [[ ! -f "$WORLD" ]]; then
    echo "ERROR: world not found: $WORLD" >&2
    exit 1
fi
if grep -q 'model://runway' "$WORLD" && [[ ! -f "$ROOT/tools/gazebo/models/runway/model.config" ]]; then
    echo "ERROR: this world uses model://runway but tools/gazebo/models/runway is incomplete." >&2
    echo "       Did you 'git pull' after the runway model was added?" >&2
    exit 1
fi

echo "GZ_IP=$GZ_IP"
echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "WORLD=$WORLD"
echo "Starting Gazebo GUI (terminal 1). Ctrl-C to stop."

if [[ "$FOCUS" -eq 1 ]]; then
    (
        # CameraTracking advertises /gui/move_to only after the 3D view is ready.
        for _ in $(seq 1 90); do
            if gz service -l 2>/dev/null | grep -qF '/gui/move_to'; then
                sleep 1
                bash "$ROOT/tools/gz_focus_homefly.sh" || true
                exit 0
            fi
            sleep 1
        done
        echo "focus: timed out waiting for /gui/move_to (GUI may still be loading)" >&2
    ) &
    WATCHER=$!
    trap 'kill "$WATCHER" 2>/dev/null || true' EXIT
fi

gz sim -r -v4 "$WORLD"
