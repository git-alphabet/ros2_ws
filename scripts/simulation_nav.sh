#!/bin/bash
set -euo pipefail

# Launch simulator and Nav2, enabling the NeuPAN virtualenv only for the controller.

export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
OVERLAY_SETUP=${OVERLAY_SETUP:-$WS_DIR/install/setup.bash}
NAV_PARAMS_FILE=${NAV_PARAMS_FILE:-$WS_DIR/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml}
NEUPAN_VENV=${NEUPAN_VENV:-$WS_DIR/neupan_env}
NEUPAN_ACTIVATE="$NEUPAN_VENV/bin/activate"
NEUPAN_SITE_PACKAGES="$NEUPAN_VENV/lib/python3.10/site-packages"
# Optional legacy hook for external model bundles; leave empty when using packaged models.
NEUPAN_MODEL_SETUP=${NEUPAN_MODEL_SETUP:-}
RCUTILS_LOGGING_SEVERITY=${RCUTILS_LOGGING_SEVERITY:-INFO}
RQT_GRAPH=${RQT_GRAPH:-false}
RQT_GRAPH_NAMESPACE=${RQT_GRAPH_NAMESPACE:-}
RQT_GRAPH_ARGS=${RQT_GRAPH_ARGS:-}

if [[ ! -f $ROS_SETUP ]]; then
	echo "[simulation_nav.sh] Missing ROS setup: $ROS_SETUP" >&2
	exit 1
fi

if [[ ! -f $OVERLAY_SETUP ]]; then
	echo "[simulation_nav.sh] Missing workspace overlay: $OVERLAY_SETUP" >&2
	exit 1
fi

TERMINAL_CMD=${TERMINAL_CMD:-}
if [[ -n $TERMINAL_CMD ]]; then
	if ! command -v "$TERMINAL_CMD" >/dev/null 2>&1; then
		echo "[simulation_nav.sh] Requested terminal '$TERMINAL_CMD' not found." >&2
		exit 1
	fi
else
	if command -v gnome-terminal >/dev/null 2>&1; then
		TERMINAL_CMD="gnome-terminal"
	elif command -v x-terminal-emulator >/dev/null 2>&1; then
		TERMINAL_CMD="x-terminal-emulator"
	else
		echo "[simulation_nav.sh] No supported graphical terminal available." >&2
		exit 1
	fi
fi

BASE_ENV="source '$ROS_SETUP'; source '$OVERLAY_SETUP'; export RCUTILS_LOGGING_SEVERITY='$RCUTILS_LOGGING_SEVERITY'"
NEUPAN_ENV=""

controller_plugin="$(python3 - <<'PY' "$NAV_PARAMS_FILE"
import sys
from pathlib import Path

cfg_path = Path(sys.argv[1])
if not cfg_path.exists():
	sys.exit(0)

try:
	import yaml  # type: ignore
except Exception:
	sys.exit(0)

data = yaml.safe_load(cfg_path.read_text()) or {}
switches = (data.get("pb_navigation_switches") or {}).get("ros__parameters") or {}
plugin = switches.get("controller_plugin")
if isinstance(plugin, str):
	plugin = plugin.strip()
	if plugin:
		print(plugin)
PY
)"

if [[ "$controller_plugin" == "neupan_nav2_controller" ]]; then
	if [[ ! -f $NEUPAN_ACTIVATE ]]; then
		echo "[simulation_nav.sh] NeuPAN virtualenv not found at $NEUPAN_ACTIVATE" >&2
		exit 1
	fi

	NEUPAN_ENV="source '$NEUPAN_ACTIVATE'"
	if [[ -d $NEUPAN_SITE_PACKAGES ]]; then
		NEUPAN_ENV="$NEUPAN_ENV; export PYTHONPATH=\$PYTHONPATH:'$NEUPAN_SITE_PACKAGES'"
	fi

	if [[ -n $NEUPAN_MODEL_SETUP ]]; then
		if [[ -f $NEUPAN_MODEL_SETUP ]]; then
			NEUPAN_ENV="$NEUPAN_ENV; source '$NEUPAN_MODEL_SETUP'"
		else
			echo "[simulation_nav.sh] Warning: $NEUPAN_MODEL_SETUP not found; skipping model setup." >&2
		fi
	fi
else
	echo "[simulation_nav.sh] controller_plugin='${controller_plugin:-unset}'; NeuPAN virtualenv will not be activated." >&2
fi

is_truthy() {
	local value=${1:-}
	case "${value,,}" in
		1|true|yes|on) return 0 ;;
		*) return 1 ;;
	esac
}

launch_in_terminal() {
	local title="$1"
	local command="$2"
	local extra_env="$3"

	echo "[simulation_nav.sh] Launching $title: $command"

	local full_cmd="cd '$WS_DIR'; $BASE_ENV"
	if [[ -n $extra_env ]]; then
		full_cmd="$full_cmd; $extra_env"
	fi
	full_cmd="$full_cmd; $command; exec bash"

	case "$TERMINAL_CMD" in
		gnome-terminal)
			gnome-terminal --title="$title" -- bash -c "$full_cmd"
			;;
		x-terminal-emulator)
			x-terminal-emulator -T "$title" -e bash -lc "$full_cmd"
			;;
		*)
			"$TERMINAL_CMD" -T "$title" -e bash -lc "$full_cmd"
			;;
	esac
}

GAZEBO_CMD=${GAZEBO_CMD:-"ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"}
NAV_CMD=${NAV_CMD:-"ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False"}

launch_in_terminal "Gazebo Sim" "$GAZEBO_CMD" ""
sleep 1
launch_in_terminal "Nav" "$NAV_CMD" "$NEUPAN_ENV"

if is_truthy "$RQT_GRAPH"; then
	rqt_env=""
	if [[ -n $RQT_GRAPH_NAMESPACE ]]; then
		rqt_env="export ROS_NAMESPACE='$RQT_GRAPH_NAMESPACE'"
	fi
	rqt_cmd="rqt_graph"
	if [[ -n $RQT_GRAPH_ARGS ]]; then
		rqt_cmd="$rqt_cmd $RQT_GRAPH_ARGS"
	fi
	launch_in_terminal "rqt_graph" "$rqt_cmd" "$rqt_env"
fi
