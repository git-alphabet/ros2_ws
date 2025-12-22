#!/usr/bin/env bash
set -euo pipefail

# One-time environment bootstrap for NeuPAN python deps.
# - Creates/updates ./neupan_env
# - Installs python deps from src/neupan_nav2_controller/requirements.txt
#
# Usage:
#   ./scripts/setup_neupan_env.sh
#   PYTHON_BIN=python3 ./scripts/setup_neupan_env.sh
#   VENV_DIR=./neupan_env ./scripts/setup_neupan_env.sh

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
VENV_DIR="${VENV_DIR:-$WS_DIR/neupan_env}"
REQ_FILE="$WS_DIR/src/neupan_nav2_controller/requirements.txt"
EXTRA_PKGS="${EXTRA_PKGS:-}"

if [[ ! -f "$REQ_FILE" ]]; then
  echo "[ERROR] requirements.txt not found: $REQ_FILE" >&2
  exit 1
fi

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "[ERROR] PYTHON_BIN not found: $PYTHON_BIN" >&2
  exit 1
fi

if [[ ! -d "$VENV_DIR" ]]; then
  echo "[INFO] Creating venv: $VENV_DIR"
  "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

# Ensure colcon ignores the virtualenv directory so builds skip it
COLCON_IGNORE_FILE="$VENV_DIR/COLCON_IGNORE"
if [[ ! -f "$COLCON_IGNORE_FILE" ]]; then
  echo "[INFO] Creating COLCON_IGNORE in $VENV_DIR"
  : > "$COLCON_IGNORE_FILE"
fi

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

echo "[INFO] Upgrading pip tooling"
python -m pip install -q --upgrade pip setuptools wheel

echo "[INFO] Installing NeuPAN python requirements"
# torch==...+cpu typically needs the CPU wheel index.
python -m pip install -r "$REQ_FILE" --extra-index-url https://download.pytorch.org/whl/cpu

if [[ -n "$EXTRA_PKGS" ]]; then
  echo "[INFO] Installing extra python packages: $EXTRA_PKGS"
  # shellcheck disable=SC2086
  python -m pip install $EXTRA_PKGS
fi

# Ensure ROS/colcon-related python packages available in the venv
INSTALL_ROS_PY_PKGS="${INSTALL_ROS_PY_PKGS:-1}"
if [[ "$INSTALL_ROS_PY_PKGS" == "1" ]]; then
  echo "[INFO] Installing ROS/colcon python packages (catkin_pkg)"
  python -m pip install -q catkin_pkg || true
else
  echo "[INFO] Skipping ROS/colcon python packages (INSTALL_ROS_PY_PKGS=$INSTALL_ROS_PY_PKGS)"
fi

echo "[INFO] Verifying imports & versions"
python - <<'PY'
import sys
from importlib import metadata as importlib_metadata

def version_of(module_name: str) -> str:
    try:
        m = __import__(module_name)
        v = getattr(m, '__version__', None)
        if v:
            return str(v)
    except Exception:
        pass
    try:
        return importlib_metadata.version(module_name)
    except Exception:
        return '(unknown)'

pkgs = [
  'numpy','scipy','torch','yaml','cvxpy','cvxpylayers','diffcp','ecos',
  'colorama','rich'
]
print('python', sys.version)
for p in pkgs:
    try:
        __import__(p)
        print(f'{p} {version_of(p)}')
    except Exception as e:
        print(f'{p} import failed:', repr(e))
        raise

try:
    __import__('catkin_pkg')
    print(f"catkin_pkg {version_of('catkin_pkg')}")
except Exception as e:
    print('[WARN] catkin_pkg not available:', repr(e))

print('OK')
PY

echo "[INFO] Done. Next steps:" \
  && echo "  - source $WS_DIR/install/setup.bash" \
  && echo "  - source $VENV_DIR/bin/activate" \
  && echo "  - (tip) activate venv last so python3 points to venv" \
  && echo "  - (optional) run ./scripts/complete_build.sh or ./scripts/neupan_build.sh"
#严格按照/src下放neupan插件    /scripts下放脚本