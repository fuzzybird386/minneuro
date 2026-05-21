#!/usr/bin/env bash
# Flash sysbuild output from this app directory (west must run with NCS as workspace cwd).
#
# Usage:
#   ./flash.sh
#   ./flash.sh --erase
#   ./flash.sh --runner jlink
#
# Verbose west / runner logs (do not use ./flash.sh -v; -v is a *west* flag, not jlink's):
#   WEST_VERBOSE=1 ./flash.sh --runner jlink
#
# Manual (must export APP_ROOT *before* source; run west from NCS workspace root):
#   export APP_ROOT="$PWD"
#   source ./ncs_env.sh
#   cd "$NCS_ROOT"
#   west -v flash -d "$APP_ROOT/build" --runner jlink
#
# Prerequisites: ./build.sh once; J-Link / Nordic probe connected.

set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${APP_ROOT}/ncs_env.sh"

BUILD_DIR="${APP_ROOT}/build"
if [[ ! -d "${BUILD_DIR}" ]]; then
	echo "error: no build directory ${BUILD_DIR}. Run ./build.sh first." >&2
	exit 1
fi
if [[ ! -f "${BUILD_DIR}/build.ninja" && ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
	echo "error: ${BUILD_DIR} does not look like a west/cmake build. Run ./build.sh first." >&2
	exit 1
fi

if [[ "${WEST_VERBOSE:-0}" == 1 ]]; then
	(cd "${NCS_ROOT}" && exec "${PY}" -m west -v flash -d "${BUILD_DIR}" "$@")
else
	(cd "${NCS_ROOT}" && exec "${PY}" -m west flash -d "${BUILD_DIR}" "$@")
fi
