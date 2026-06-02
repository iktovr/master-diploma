#!/usr/bin/env bash
# Wrapper for process_map.py — uses the system python3 which has
# python3-pyosmium installed via apt (see .devcontainer/Dockerfile).
set -euo pipefail

# Resolve symlinks so we find the real source directory even when Bazel
# invokes this script through a symlink in bazel-bin/.
REAL_SCRIPT="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${REAL_SCRIPT}")"
PY_SCRIPT="${SCRIPT_DIR}/process_map.py"
export BUILD_WORKING_DIRECTORY

exec python3 "${PY_SCRIPT}" "$@"
