#!/usr/bin/env bash
# Environment profiles for the M20 stack: WHERE the robot is (M20_ENV) and WHERE commands run (M20_PLATFORM).
# Source it, then every tool reads the exported variables:   source tools/dev/env.sh [emulator|robot|auto]
#
#   M20_ENV=emulator   BrightAI's protocol emulator on this machine (127.0.0.1:30000, perfect "localization")
#   M20_ENV=robot      the real M20 on the wire (AOS 10.21.31.103:30000; the laptop at 10.21.31.100/24)
#   M20_ENV=auto       (default) robot if 10.21.31.103 answers a ping within 1 s, else emulator
#   M20_PLATFORM       linux | wsl | windows (Git Bash) | mac, detected; Windows users run through WSL (m20.ps1)
set -u
_env="${1:-${M20_ENV:-auto}}"

case "$(uname -s)" in
  Linux)  if grep -qi microsoft /proc/version 2>/dev/null; then M20_PLATFORM=wsl; else M20_PLATFORM=linux; fi ;;
  Darwin) M20_PLATFORM=mac ;;
  MINGW*|MSYS*|CYGWIN*) M20_PLATFORM=windows ;;
  *)      M20_PLATFORM=unknown ;;
esac

if [ "$_env" = auto ]; then
  if ping -c 1 -W 1 10.21.31.103 >/dev/null 2>&1; then _env=robot; else _env=emulator; fi
fi
case "$_env" in
  emulator)
    M20_ROBOT_IP=127.0.0.1; M20_ROBOT_PORT=30000
    M20_VX_MAX=2.1; M20_VY_MAX=0.62; M20_WZ_MAX=1.8          # the emulator's own speed table
    ;;
  robot)
    M20_ROBOT_IP=10.21.31.103; M20_ROBOT_PORT=30000
    M20_VX_MAX="${M20_VX_MAX:-2.1}"; M20_VY_MAX="${M20_VY_MAX:-0.62}"; M20_WZ_MAX="${M20_WZ_MAX:-1.8}"   # measured 2026-09-29; redo at step 2
    ;;
  *) echo "env.sh: unknown M20_ENV '$_env' (emulator | robot | auto)" >&2; return 2 2>/dev/null || exit 2 ;;
esac
M20_ENV="$_env"
export M20_ENV M20_PLATFORM M20_ROBOT_IP M20_ROBOT_PORT M20_VX_MAX M20_VY_MAX M20_WZ_MAX
[ -n "${M20_ENV_QUIET:-}" ] || echo "[env] M20_ENV=$M20_ENV robot=$M20_ROBOT_IP:$M20_ROBOT_PORT platform=$M20_PLATFORM"
