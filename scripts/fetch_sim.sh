#!/usr/bin/env bash
set -euo pipefail

SIM_DIR=".sim"
SIM_BIN="$SIM_DIR/autodrive_simulator/AutoDRIVE Simulator.x86_64"
URL="https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2026-icra/autodrive_devkit.zip"

if [ -x "$SIM_BIN" ]; then
  exit 0
fi

mkdir -p "$SIM_DIR"
TMP_ZIP="$(mktemp --suffix=.zip)"
trap 'rm -f "$TMP_ZIP"' EXIT

echo "Downloading AutoDRIVE simulator..."
wget -O "$TMP_ZIP" "$URL"

echo "Extracting to $SIM_DIR/..."
unzip -q "$TMP_ZIP" -d "$SIM_DIR"

echo "Simulator ready at $SIM_BIN"
