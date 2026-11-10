#!/usr/bin/env bash
# One-time setup of armdog (the Jetson Orin inside Dog1) for this stack, run there over ssh. No sudo: Docker 26 is
# installed and the user is in the docker group, but there is no compose plugin and /etc is off limits
# (docs/REAL_ROBOT.md: nothing under /etc, no system changes on the robot's computers).
#
#   bash tools/robot/armdog_setup.sh          checks arm64 + disk, installs the compose v2 plugin for this user, verifies
#
# After it: build the two images natively (each once, ~20 and ~10 min):
#   docker build -f docker/Dockerfile -t m20_autonomy:latest .
#   docker build -f docker/Dockerfile.rslidar -t m20_lidar:latest .
set -euo pipefail
COMPOSE_VERSION="${COMPOSE_VERSION:-v2.29.7}"
PLUGIN_DIR="$HOME/.docker/cli-plugins"

[ "$(uname -m)" = "aarch64" ] || { echo "armdog_setup: this is $(uname -m), not aarch64; run it ON armdog"; exit 1; }
FREE_GB=$(df -BG --output=avail / | tail -1 | tr -dc '0-9')
[ "$FREE_GB" -ge 8 ] || { echo "armdog_setup: only ${FREE_GB} GB free on /; the two images need ~8 GB"; exit 1; }
docker info >/dev/null 2>&1 || { echo "armdog_setup: docker is not usable by $USER (docker group?)"; exit 1; }
echo "armdog_setup: aarch64, ${FREE_GB} GB free, docker $(docker version --format '{{.Server.Version}}')"

if docker compose version >/dev/null 2>&1; then
  echo "armdog_setup: compose plugin present: $(docker compose version)"
else
  # the plugin binary for this user only; the release's checksums.txt verifies it (no hash hard-coded here)
  mkdir -p "$PLUGIN_DIR"
  BASE="https://github.com/docker/compose/releases/download/$COMPOSE_VERSION"
  TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
  curl -fsSL "$BASE/docker-compose-linux-aarch64" -o "$TMP/docker-compose"
  curl -fsSL "$BASE/checksums.txt" -o "$TMP/checksums.txt"
  (cd "$TMP" && grep ' docker-compose-linux-aarch64$' checksums.txt | sed 's/docker-compose-linux-aarch64/docker-compose/' | sha256sum -c -)
  install -m 0755 "$TMP/docker-compose" "$PLUGIN_DIR/docker-compose"
  echo "armdog_setup: installed $(docker compose version)"
fi
docker image inspect ros:humble-ros-base >/dev/null 2>&1 && echo "armdog_setup: ros:humble-ros-base present (arm64)" \
  || echo "armdog_setup: ros:humble-ros-base not pulled yet; the image builds will pull it"
echo "armdog_setup: done. Next: the two docker builds above, then tools/lidar/lidar_up.sh --check"
