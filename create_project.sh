#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE_DIR="$SCRIPT_DIR/uros_example"
OLD_NAME="uros_example"
USAGE="usage: create_project.sh <workspace> <project_name> <humble|jazzy>"

if [[ -t 1 ]]; then
  GREEN=$'\033[1;32m'; RED=$'\033[1;31m'; RESET=$'\033[0m'
else
  GREEN=""; RED=""; RESET=""
fi
fail() { echo "${RED}Failed:${RESET} $*" >&2; exit 1; }

[[ $# -eq 3 ]] || fail "expected 3 arguments, got $#. $USAGE"
WS="$1"; NEW_NAME="$2"; DISTRO="$3"
[[ "$DISTRO" == humble || "$DISTRO" == jazzy ]] || fail "distro must be humble or jazzy, got '$DISTRO'"

[[ "$NEW_NAME" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]] ||
  fail "'$NEW_NAME' is not a valid name (start with a letter; letters, digits, underscores)"
[[ -d "$TEMPLATE_DIR" ]] || fail "template folder not found: $TEMPLATE_DIR"

FW_DIR="$(realpath -m "$WS")/firmware"
DEST="$FW_DIR/$NEW_NAME"
[[ ! -e "$DEST" ]] || fail "$DEST already exists"
case "$DEST/" in "$SCRIPT_DIR"/*) fail "workspace must be outside the template repository" ;; esac

trap 'rm -rf "$DEST"; fail "command on line $LINENO did not finish"' ERR

mkdir -p "$FW_DIR"
touch "$FW_DIR/COLCON_IGNORE"
cp -a "$TEMPLATE_DIR" "$DEST"
for f in .project .cproject "$OLD_NAME.ioc" "$OLD_NAME.launch"; do
  if [[ -f "$DEST/$f" ]]; then sed -i "s/$OLD_NAME/$NEW_NAME/g" "$DEST/$f"; fi
done
sed -i "s/micro_ros_static_library_builder:humble/micro_ros_static_library_builder:$DISTRO/g" "$DEST/.cproject"
mv "$DEST/$OLD_NAME.ioc" "$DEST/$NEW_NAME.ioc"
mv "$DEST/$OLD_NAME.launch" "$DEST/$NEW_NAME.launch"
rm -rf "$DEST/Debug" "$DEST/Release"

echo "${GREEN}Done:${RESET} $DEST ($DISTRO)"
