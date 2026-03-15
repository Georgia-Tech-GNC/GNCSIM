#!/bin/zsh
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
SRC_FILE="$ROOT_DIR/src/DATCOM/datcom.f"
OUT_FILE="$ROOT_DIR/src/DATCOM/datcom_macos"

if ! command -v gfortran >/dev/null 2>&1; then
  echo "gfortran not found in PATH." >&2
  exit 1
fi

SDKROOT_VALUE=$(xcrun --show-sdk-path)

echo "Building DATCOM for macOS..."
echo "  source: $SRC_FILE"
echo "  output: $OUT_FILE"
echo "  SDKROOT: $SDKROOT_VALUE"

SDKROOT="$SDKROOT_VALUE" gfortran \
  -O2 \
  -ffree-form \
  -std=legacy \
  -fallow-argument-mismatch \
  -o "$OUT_FILE" \
  "$SRC_FILE"

echo "Build complete."
file "$OUT_FILE"
