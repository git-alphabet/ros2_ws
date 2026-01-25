#!/usr/bin/env bash
# Safe build script: deactivates conda base, clears LD_LIBRARY_PATH,
# sources ROS, and runs colcon build with any passed arguments.

set -euo pipefail

# Deactivate conda if active
if command -v conda >/dev/null 2>&1; then
  # Try to deactivate base environment without erroring if not active
  source "$(conda info --base 2>/dev/null)/etc/profile.d/conda.sh" >/dev/null 2>&1 || true
  conda deactivate >/dev/null 2>&1 || true
fi

# Clear LD_LIBRARY_PATH to prefer system libs
unset LD_LIBRARY_PATH || true

# Ensure system python is used (prefer /usr/bin)
export PATH="/usr/bin:${PATH}"

# Source ROS 2 environment (temporarily disable -u to avoid unbound vars in setup)
if [ -f "/opt/ros/humble/setup.bash" ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

# Run colcon with forwarded args (or default)
if [ "$#" -eq 0 ]; then
  colcon build --event-handlers console_direct+
else
  colcon build "$@"
fi
