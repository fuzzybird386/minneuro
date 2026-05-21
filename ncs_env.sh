# Sourced by build.sh / flash.sh — Nordic NCS venv (macOS nrfutil default paths).
# shellcheck shell=bash
#
# Do not use set -e here: interactive zsh sources this file; a failing : "${APP_ROOT:?...}"
# under errexit closes the whole terminal. build.sh / flash.sh keep their own strict mode.

if [[ -z "${APP_ROOT:-}" ]]; then
	echo "error: APP_ROOT must be set before sourcing ncs_env.sh (e.g. export APP_ROOT=\"\$PWD\")." >&2
	return 1 2>/dev/null || exit 1
fi

NCS_ROOT="${NCS_ROOT:-/opt/nordic/ncs/v3.3.0}"
export ZEPHYR_BASE="${ZEPHYR_BASE:-${NCS_ROOT}/zephyr}"
BOARD="${BOARD:-minneuro_rhs_devkit/nrf54l15/cpuapp/ns}"

TOOL_OPT=""
for cand in /opt/nordic/ncs/toolchains/*/opt; do
	if [[ -d "${cand}/zephyr-sdk" ]]; then
		TOOL_OPT="${cand}"
		break
	fi
done
if [[ -z "${TOOL_OPT}" ]]; then
	echo "error: could not find /opt/nordic/ncs/toolchains/*/opt/zephyr-sdk (install NCS toolchain via nrfutil sdk-manager)." >&2
	return 2 2>/dev/null || exit 2
fi

export ZEPHYR_SDK_INSTALL_DIR="${TOOL_OPT}/zephyr-sdk"
export PATH="${TOOL_OPT}/dtc/bin:${TOOL_OPT}/zephyr-sdk/arm-zephyr-eabi/bin:${PATH}"

PY312="${TOOL_OPT}/python@3.12/bin/python3.12"
PY312_ALT="${TOOL_OPT}/python@3.12/bin/python3"
if [[ -x "${PY312}" ]]; then
	PY="${PY312}"
elif [[ -x "${PY312_ALT}" ]]; then
	PY="${PY312_ALT}"
else
	echo "error: toolchain Python 3.12 not found under ${TOOL_OPT}/python@3.12/bin" >&2
	return 1 2>/dev/null || exit 1
fi

export WEST_PYTHON="${PY}"
# shellcheck source=/dev/null
if ! source "${ZEPHYR_BASE}/zephyr-env.sh"; then
	echo "error: failed to source ${ZEPHYR_BASE}/zephyr-env.sh" >&2
	return 1 2>/dev/null || exit 1
fi

if ! "${PY}" -m west --version &>/dev/null; then
	echo "error: west is not installed for this Python. Run:" >&2
	echo "  ${PY} -m pip install west" >&2
	return 1 2>/dev/null || exit 1
fi
