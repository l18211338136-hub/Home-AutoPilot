#!/usr/bin/env bash
# Focus the Gazebo Sim 8 GUI camera on the HomeFly vehicle.
#
# Root cause this works around: the default GUI camera pose comes from
# ~/.gz/sim/<ver>/gui.config (roughly "-6 0 6 0 0.5 0", i.e. ~8 m away looking
# down at the origin) and does NOT adapt to the world bounding box. The
# verbatim ArduPilot airfield is a 1500 x 100 m PBR runway, so the 0.3 m
# dark-grey Iris is a few low-contrast pixels and looks "missing" even though
# it is present in the Entity Tree and the sim clock is running.
#
# This helper calls the CameraTracking GUI plugin's /gui/move_to service, which
# frames the named entity automatically (0.5 s smooth animation). No world SDF
# edits and no custom gui.config are needed, so the default Entity Tree /
# play-pause / view-angle plugins are preserved.
#
# Usage, in ANY terminal AFTER terminal 1 has opened the GUI window:
#     bash tools/gz_focus_homefly.sh                 # frame the vehicle once
#     bash tools/gz_focus_homefly.sh --follow       # keep following while flying
#     bash tools/gz_focus_homefly.sh homefly        # explicit entity name
#     bash tools/gz_focus_homefly.sh --help
#
# GZ_IP defaults to 127.0.0.1 (WSLg multicast discovery workaround). Terminal 1
# must have exported the same value, or the GUI and this client cannot see each
# other's services.

set -euo pipefail

ENTITY="homefly"
SERVICE="/gui/move_to"

for arg in "$@"; do
    case "$arg" in
        --follow) SERVICE="/gui/follow" ;;
        -h|--help)
            echo "Usage: bash tools/gz_focus_homefly.sh [--follow] [ENTITY]"
            echo "  --follow   keep the camera following the vehicle (live stick input)"
            echo "  ENTITY     model name to frame, default: homefly"
            echo "  Requires a running Gazebo Sim GUI with GZ_IP=127.0.0.1."
            exit 0
            ;;
        *) ENTITY="$arg" ;;
    esac
done

# WSL2: gz-transport discovery over multicast is unreliable, so the GUI and
# this client must agree on the loopback IP.
export GZ_IP="${GZ_IP:-127.0.0.1}"

if ! command -v gz >/dev/null 2>&1; then
    echo "ERROR: 'gz' CLI not found in PATH." >&2
    exit 1
fi

# CameraTracking advertises these services only after the 3D view finishes
# initialising. Give an actionable hint instead of a bare transport error.
if ! gz service -l 2>/dev/null | grep -qF "$SERVICE"; then
    echo "ERROR: ${SERVICE} is not advertised." >&2
    echo "  - Is the Gazebo GUI window (terminal 1) fully up, with the 3D view loaded?" >&2
    echo "  - Was GZ_IP=${GZ_IP} also exported in terminal 1?" >&2
    echo "  - If the viewport is blank, restart terminal 1 with LIBGL_ALWAYS_SOFTWARE=1." >&2
    exit 1
fi

echo "Focusing GUI camera on '${ENTITY}' via ${SERVICE} (GZ_IP=${GZ_IP}) ..."
gz service -s "$SERVICE" \
    --reqtype gz.msgs.StringMsg \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    --req "data: \"${ENTITY}\""

echo "Done - the camera animates for ~0.5 s. Use --follow when flying with the GCS."
