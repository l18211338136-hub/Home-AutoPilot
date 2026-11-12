#!/usr/bin/env bash
# HomeFly <-> Gazebo SITL one-shot runner.
#
# Target: WSL2 Ubuntu (or any Linux). Gazebo server, the Python bridge and the
# flight controller MUST all live in the same network namespace: WSL2's
# localhostForwarding only forwards TCP, so a Windows .exe talking UDP over
# 127.0.0.1 to a WSL2 bridge does not work. Always rebuild natively here.
#
#   bash tools/run_wsl_sitl.sh                          # mock plant, 2000 frames
#   bash tools/run_wsl_sitl.sh --backend gz            # real Gazebo, headless
#   bash tools/run_wsl_sitl.sh --steps 1000 --failsafe
#   BUILD_DIR=build/wsl PYTHON=python3 bash tools/run_wsl_sitl.sh --build
#
# Env overrides: BUILD_DIR (default build/wsl), PYTHON (default python3).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
# Keep every path RELATIVE. Native Windows binaries invoked from Git Bash
# mangle POSIX absolute paths (/d/... becomes D:\d\...); relative paths work
# on both WSL/Linux and Git Bash.
BUILD_DIR="${BUILD_DIR:-build/wsl}"
PYTHON="${PYTHON:-python3}"
BACKEND="mock"
STEPS=2000
WORLD="tools/gazebo/homefly_quad.world"
BUILD_MODE="auto"            # auto | yes | no
FAILSAFE=0

usage() {
	cat <<'EOF'
Usage: run_wsl_sitl.sh [options]

  --backend mock|gz|classic   simulation source (default: mock)
  --steps N                   IMU/motor frames (default: 2000)
  --world PATH                world SDF for --backend gz
  --build                     force a native rebuild
  --no-build                  never build; fail if the binary is missing
  --failsafe                  drop the IMU at 40% and verify protection
  -h, --help                  show this message

Artifacts go to $BUILD_DIR/sitl/: bridge.log, gz.log, sitl.hfl.

NOTE: --backend gz/classic needs a real Gazebo plus its Python bindings.
The bundled SITL bridge is an adapter stub: verify topic names, message
fields and Actuators.velocity units against your Gazebo release first.
EOF
}

die() { echo "run_wsl_sitl: $*" >&2; exit 1; }

while (($#)); do
	case "$1" in
		--backend|--steps|--world)
			(($# >= 2)) || die "$1 needs a value"
			case "$1" in
				--backend) BACKEND="$2" ;;
				--steps) STEPS="$2" ;;
				--world) WORLD="$2" ;;
			esac
			shift 2 ;;
		--build) BUILD_MODE="yes"; shift ;;
		--no-build) BUILD_MODE="no"; shift ;;
		--failsafe) FAILSAFE=1; shift ;;
		-h|--help) usage; exit 0 ;;
		*) die "unknown option: $1 (try --help)" ;;
	esac
done

case "$BACKEND" in mock|gz|classic) ;; *) die "--backend must be mock|gz|classic" ;; esac
[[ "$STEPS" =~ ^[0-9]+$ ]] || die "--steps must be a positive integer"
(("$STEPS" > 0)) || die "--steps must be a positive integer"

"$PYTHON" -V >/dev/null 2>&1 || die "Python not found: $PYTHON (set PYTHON=...)"

FC="$BUILD_DIR/src/applications/linux_native/homefly_linux_native"
# On a Windows host (Git Bash) the native build produces a .exe; this only
# exists so the very same script can smoke-test itself without WSL.
if [[ ! -x "$FC" && -x "$FC.exe" ]]; then FC="$FC.exe"; fi

if [[ "$BUILD_MODE" == "yes" || ( "$BUILD_MODE" == "auto" && ! -x "$FC" ) ]]; then
	command -v cmake >/dev/null || die "cmake not found; sudo apt install cmake ninja-build g++"
	command -v ninja >/dev/null || die "ninja not found; sudo apt install ninja-build"
	echo "==> building HomeFly into $BUILD_DIR"
	cmake -S . -B "$BUILD_DIR" -G Ninja -DHOMEFLY_PLATFORM=linux
	cmake --build "$BUILD_DIR"
	if [[ ! -x "$FC" && -x "$FC.exe" ]]; then FC="$FC.exe"; fi
fi
[[ -x "$FC" ]] || die "flight controller binary missing: $FC (run with --build)"

LOG_DIR="$BUILD_DIR/sitl"
mkdir -p "$LOG_DIR"
HFL="$LOG_DIR/sitl.hfl"
rm -f "$HFL"

echo "==> wire protocol self-test"
"$PYTHON" "tools/gazebo_bridge.py" --self-test

PIDS=()
cleanup() {
	local rc=$?
	trap - EXIT INT TERM
	for pid in "${PIDS[@]}"; do
		kill "$pid" 2>/dev/null || true
	done
	exit "$rc"
}
trap cleanup EXIT INT TERM

if [[ "$BACKEND" != "mock" ]]; then
	command -v gz >/dev/null || die "gz not found; install Gazebo in this WSL distro"
	[[ -f "$WORLD" ]] || die "world not found: $WORLD"
	echo "==> starting Gazebo (headless): $WORLD"
	gz sim -s -v4 "$WORLD" > "$LOG_DIR/gz.log" 2>&1 &
	PIDS+=("$!")
	sleep 2
	if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
		echo "--- gz.log ---" >&2
		cat "$LOG_DIR/gz.log" >&2
		die "Gazebo exited early; check plugin names / SDF for your release"
	fi
	echo "    (gz.log: $LOG_DIR/gz.log)"
fi

echo "==> starting bridge (backend=$BACKEND, steps=$STEPS)"
"$PYTHON" "tools/gazebo_bridge.py" \
	--backend "$BACKEND" --steps "$STEPS" \
	> "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
PIDS+=("$BRIDGE_PID")
sleep 0.4

FC_ARGS=(--steps "$STEPS" --gazebo --flight-log "$HFL")
(("$FAILSAFE")) && FC_ARGS+=(--failsafe)

echo "==> starting flight controller: $FC"
set +e
"$FC" "${FC_ARGS[@]}"
FC_RC=$?
wait "$BRIDGE_PID"
BRIDGE_RC=$?
set -e

echo ""
echo "--- bridge.log ---"
cat "$LOG_DIR/bridge.log"
echo "-------------------"

if [[ -f "$HFL" ]]; then
	echo "==> verifying flight log"
	"$PYTHON" "tools/hfl_dump.py" "$HFL" --strict --tail 3
fi

echo "==> flight controller rc=$FC_RC, bridge rc=$BRIDGE_RC"
if ((FC_RC != 0 || BRIDGE_RC != 0)); then
	die "SITL run FAILED (see $LOG_DIR)"
fi
echo "==> SITL run PASS (artifacts in $LOG_DIR)"
