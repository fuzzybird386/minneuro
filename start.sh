#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TOOLCHAIN_ROOT="${TOOLCHAIN_ROOT:-/opt/nordic/ncs/toolchains/322ac893fe}"
SDK_ROOT="${SDK_ROOT:-/opt/nordic/ncs/v3.2.0}"
BUILD_DIR="${BUILD_DIR:-build}"
RUNNER="${RUNNER:-jlink}"
JLINK_DEVICE="${JLINK_DEVICE:-nRF54L15_M33}"
JLINK_SPEED="${JLINK_SPEED:-4000}"
JLINK_GDB_PORT="${JLINK_GDB_PORT:-2331}"
JLINK_RTT_PORT="${JLINK_RTT_PORT:-19021}"
JLINK_GDBSERVER="${JLINK_GDBSERVER:-JLinkGDBServer}"
JLINK_SERIAL="${JLINK_SERIAL:-}"
DO_FLASH=0
DO_RESET=0
SKIP_REBUILD=1

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Open RTT output immediately. Use -f to flash first.

Options:
  -d, --build-dir DIR   Build directory to use (default: ${BUILD_DIR})
  -f, --flash           Flash before opening RTT
  -r, --reset           Reset once before opening RTT
  --rebuild             Allow west to rebuild before flash/rtt
  -h, --help            Show this help

Environment overrides:
  TOOLCHAIN_ROOT        Default: ${TOOLCHAIN_ROOT}
  SDK_ROOT              Default: ${SDK_ROOT}
  RUNNER                Default: ${RUNNER}
  BUILD_DIR             Default: ${BUILD_DIR}
  JLINK_DEVICE          Default: ${JLINK_DEVICE}
  JLINK_SPEED           Default: ${JLINK_SPEED}
  JLINK_GDB_PORT        Default: ${JLINK_GDB_PORT}
  JLINK_RTT_PORT        Default: ${JLINK_RTT_PORT}
  JLINK_GDBSERVER       Default: ${JLINK_GDBSERVER}
  JLINK_SERIAL          Optional explicit J-Link serial number
  RTT_SKIP_SET_ADDR=1 Skip GDB \"monitor exec SetRTTAddr\"; use if auto-scan works
  RTT_GDB_BIN           Override arm-zephyr-eabi-gdb (default: from PATH after PATH tweak)

Examples:
  ./start.sh
  ./start.sh -f
  ./start.sh -r
  ./start.sh -f -r
  ./start.sh -d build_debug
  BUILD_DIR=build_debug ./start.sh -f
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--build-dir)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 1; }
      BUILD_DIR="$2"
      shift 2
      ;;
    -f|--flash)
      DO_FLASH=1
      shift
      ;;
    -r|--reset)
      DO_RESET=1
      shift
      ;;
    --rebuild)
      SKIP_REBUILD=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

WEST_BIN="${TOOLCHAIN_ROOT}/bin/west"
NCS_NRFUTIL_BIN="${TOOLCHAIN_ROOT}/nrfutil/bin"
NCS_NRFUTIL_HOME="${TOOLCHAIN_ROOT}/nrfutil/home"
NCS_NRFUTIL_HOME_BIN="${NCS_NRFUTIL_HOME}/bin"

export ZEPHYR_BASE="${SDK_ROOT}/zephyr"
export NRFUTIL_HOME="${NCS_NRFUTIL_HOME}"
export PATH="${TOOLCHAIN_ROOT}/bin:${NCS_NRFUTIL_BIN}:${NCS_NRFUTIL_HOME_BIN}:${PATH}"

server_pid=""

cleanup() {
  if [[ -n "${server_pid}" ]] && kill -0 "${server_pid}" 2>/dev/null; then
    kill "${server_pid}" 2>/dev/null || true
    wait "${server_pid}" 2>/dev/null || true
  fi
}

detect_jlink_serial() {
  if [[ -n "${JLINK_SERIAL}" ]]; then
    printf '%s\n' "${JLINK_SERIAL}"
    return 0
  fi

  if command -v nrfutil >/dev/null 2>&1; then
    local serial
    serial="$(nrfutil device list 2>/dev/null | awk 'NR==1 {print $1}')"
    if [[ -n "${serial}" ]]; then
      printf '%s\n' "${serial}"
      return 0
    fi
  fi

  if command -v nrfjprog >/dev/null 2>&1; then
    local serial
    serial="$(nrfjprog --ids 2>/dev/null | awk 'NF {print; exit}')"
    if [[ -n "${serial}" ]]; then
      printf '%s\n' "${serial}"
      return 0
    fi
  fi

  return 1
}

wait_for_port() {
  local host="$1"
  local port="$2"
  local tries="${3:-100}"
  local i

  for ((i=0; i<tries; i++)); do
    if nc -z "${host}" "${port}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done

  return 1
}

rtt_resolve_zephyr_elf() {
  local base="${SCRIPT_DIR}/${BUILD_DIR}"
  local cands=(
    "${base}/minneuro_mvp_band_core-main/zephyr/zephyr.elf"
    "${base}/zephyr/zephyr.elf"
  )
  local f
  for f in "${cands[@]}"; do
    if [[ -f "$f" ]]; then
      printf '%s\n' "$f"
      return 0
    fi
  done
  return 1
}

prepend_zephyr_gnu_bin() {
  local cand
  for cand in /opt/nordic/ncs/toolchains/*/opt/zephyr-sdk/arm-zephyr-eabi/bin; do
    if [[ -x "${cand}/arm-zephyr-eabi-nm" && -x "${cand}/arm-zephyr-eabi-gdb" ]]; then
      export PATH="${cand}:${PATH}"
      return 0
    fi
  done
  return 1
}

rtt_set_control_block_via_gdb() {
  local elf="$1"
  local addr

  addr="$(arm-zephyr-eabi-nm "${elf}" 2>/dev/null | awk '$NF == "_SEGGER_RTT" { print "0x" $1; exit }')"
  if [[ -z "${addr}" ]]; then
    echo "warning: _SEGGER_RTT not found in ${elf}; RTT auto-scan must succeed or set address manually." >&2
    return 1
  fi

  local gdb_bin="${RTT_GDB_BIN:-arm-zephyr-eabi-gdb}"
  if ! command -v "${gdb_bin}" >/dev/null 2>&1; then
    echo "warning: ${gdb_bin} not in PATH (need Zephyr SDK GNU tools). RTT CB address is ${addr} — use SEGGER RTT Viewer manual CB or GDB: monitor exec SetRTTAddr ${addr}" >&2
    return 1
  fi

  echo "Pointing J-Link RTT at control block ${addr} (monitor exec SetRTTAddr)."
  "${gdb_bin}" -nx -nh -batch \
    -ex "target extended-remote localhost:${JLINK_GDB_PORT}" \
    -ex "monitor exec SetRTTAddr ${addr}" \
    -ex disconnect \
    -ex q >/dev/null 2>&1 || {
    echo "warning: GDB SetRTTAddr failed; RTT address is ${addr} — try SEGGER RTT Viewer or connect GDB once manually." >&2
    return 1
  }
  echo "Configured J-Link RTT with SetRTTAddr ${addr}."
}

reset_device() {
  local serial="$1"

  if command -v nrfutil >/dev/null 2>&1; then
    nrfutil device reset \
      --serial-number "${serial}" \
      --family nrf54l \
      --reset-kind RESET_DEFAULT
    return 0
  fi

  if command -v nrfjprog >/dev/null 2>&1; then
    nrfjprog --snr "${serial}" --family UNKNOWN --reset
    return 0
  fi

  echo "Neither nrfutil nor nrfjprog is available for reset." >&2
  return 1
}

trap cleanup EXIT INT TERM

if [[ ! -x "${WEST_BIN}" ]]; then
  echo "west not found: ${WEST_BIN}" >&2
  exit 1
fi

if [[ ! -d "${ZEPHYR_BASE}" ]]; then
  echo "ZEPHYR_BASE not found: ${ZEPHYR_BASE}" >&2
  exit 1
fi

if [[ ! -d "${SCRIPT_DIR}/${BUILD_DIR}" ]]; then
  echo "Build directory not found: ${SCRIPT_DIR}/${BUILD_DIR}" >&2
  exit 1
fi

COMMON_ARGS=(-d "${SCRIPT_DIR}/${BUILD_DIR}")
if [[ "${SKIP_REBUILD}" -eq 1 ]]; then
  COMMON_ARGS+=(--skip-rebuild)
fi

echo "Using build dir : ${BUILD_DIR}"
echo "Using runner    : ${RUNNER}"
echo "Flash first     : ${DO_FLASH}"
echo "Reset first     : ${DO_RESET}"
echo "SDK root        : ${SDK_ROOT}"
echo "Toolchain root  : ${TOOLCHAIN_ROOT}"

if [[ "${DO_FLASH}" -eq 1 ]]; then
  echo
  echo "Flashing..."
  "${WEST_BIN}" flash "${COMMON_ARGS[@]}" -r "${RUNNER}"
fi

if ! command -v nc >/dev/null 2>&1; then
  echo "nc not found in PATH; cannot open RTT stream." >&2
  exit 1
fi

select_arg="usb"
if detected_serial="$(detect_jlink_serial)"; then
  select_arg="usb=${detected_serial}"
  echo
  echo "Using J-Link S/N: ${detected_serial}"
fi

if [[ "${DO_RESET}" -eq 1 ]]; then
  if [[ -z "${detected_serial:-}" ]]; then
    echo "Could not detect a J-Link serial number for reset." >&2
    exit 1
  fi

  echo
  echo "Resetting device..."
  reset_device "${detected_serial}"
  sleep 0.2
fi

prepend_zephyr_gnu_bin || true

zephyr_elf=""
if zephyr_elf="$(rtt_resolve_zephyr_elf)"; then
  echo "Zephyr ELF      : ${zephyr_elf}"
else
  echo "warning: could not find zephyr.elf under ${SCRIPT_DIR}/${BUILD_DIR} (needed for RTT SetRTTAddr)." >&2
fi

echo
echo "Starting J-Link RTT server..."
# Do not use -singlerun: we connect GDB briefly to SetRTTAddr; singlerun would exit the server.
"${JLINK_GDBSERVER}" \
  -select "${select_arg}" \
  -port "${JLINK_GDB_PORT}" \
  -if swd \
  -speed "${JLINK_SPEED}" \
  -device "${JLINK_DEVICE}" \
  -silent \
  -nogui \
  -nohalt \
  -rtttelnetport "${JLINK_RTT_PORT}" &
server_pid=$!

if ! wait_for_port localhost "${JLINK_GDB_PORT}" 150; then
  echo "J-Link GDB server did not open port ${JLINK_GDB_PORT}." >&2
  exit 1
fi

if [[ "${RTT_SKIP_SET_ADDR:-0}" != "1" && -n "${zephyr_elf}" ]]; then
  sleep 0.2
  rtt_set_control_block_via_gdb "${zephyr_elf}" || true
else
  if [[ "${RTT_SKIP_SET_ADDR:-0}" == "1" ]]; then
    echo "RTT_SKIP_SET_ADDR=1: skipping GDB SetRTTAddr (J-Link will only auto-scan RAM)."
  fi
fi

if ! wait_for_port localhost "${JLINK_RTT_PORT}" 150; then
  echo "RTT server did not open port ${JLINK_RTT_PORT}." >&2
  exit 1
fi

echo
echo "Opening RTT on localhost:${JLINK_RTT_PORT}..."
echo "Press Ctrl+C to exit RTT."
nc localhost "${JLINK_RTT_PORT}"
