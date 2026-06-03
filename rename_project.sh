#!/usr/bin/env bash
#
# rename_project.sh — turn this template into your own STM32CubeIDE project.
#
# Usage:
#   ./rename_project.sh <new_project_name>
#
# Renames the CubeIDE project — the folder, the .ioc/.launch files, and every
# internal reference — from its current name to <new_project_name>, and removes
# stale build output. Safe to re-run: it detects the current name from the .ioc
# file, so you can rename again later.
#
set -euo pipefail

# --- locate this script's directory (the template root holding the project) --
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- validate the requested name ---------------------------------------------
if [[ $# -ne 1 ]]; then
  echo "Usage: $(basename "$0") <new_project_name>" >&2
  exit 1
fi
NEW_NAME="$1"
# The name becomes the build artifact name and a Docker volume path, so it must
# be a plain identifier: start with a letter, then letters/digits/underscores.
if [[ ! "$NEW_NAME" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]]; then
  echo "Error: project name must match ^[A-Za-z][A-Za-z0-9_]*\$" >&2
  echo "       (start with a letter; only letters, digits, underscores). Got: '$NEW_NAME'" >&2
  exit 1
fi

# --- find the current project (the directory containing the single .ioc) ------
IOC_PATH="$(find "$SCRIPT_DIR" -maxdepth 2 -name '*.ioc' | head -n 1 || true)"
if [[ -z "$IOC_PATH" ]]; then
  echo "Error: no .ioc file found under $SCRIPT_DIR — run this from the template root." >&2
  exit 1
fi
PROJECT_DIR="$(dirname "$IOC_PATH")"
OLD_NAME="$(basename "$IOC_PATH" .ioc)"

if [[ "$OLD_NAME" == "$NEW_NAME" ]]; then
  echo "Project is already named '$NEW_NAME' — nothing to do."
  exit 0
fi

echo "Renaming project '$OLD_NAME' -> '$NEW_NAME'"
echo "  location: $PROJECT_DIR"

# --- 1) rewrite every internal reference in the config files ------------------
for f in .project .cproject "$OLD_NAME.ioc" "$OLD_NAME.launch"; do
  if [[ -f "$PROJECT_DIR/$f" ]]; then
    sed -i "s/${OLD_NAME}/${NEW_NAME}/g" "$PROJECT_DIR/$f"
    echo "  updated  $f"
  fi
done

# --- 2) rename the name-bearing files ----------------------------------------
if [[ -f "$PROJECT_DIR/$OLD_NAME.ioc" ]]; then
  mv "$PROJECT_DIR/$OLD_NAME.ioc" "$PROJECT_DIR/$NEW_NAME.ioc"
  echo "  renamed  $OLD_NAME.ioc -> $NEW_NAME.ioc"
fi
if [[ -f "$PROJECT_DIR/$OLD_NAME.launch" ]]; then
  mv "$PROJECT_DIR/$OLD_NAME.launch" "$PROJECT_DIR/$NEW_NAME.launch"
  echo "  renamed  $OLD_NAME.launch -> $NEW_NAME.launch"
fi

# --- 3) drop stale build output (CubeIDE regenerates it on the next build) ----
rm -rf "$PROJECT_DIR/Debug" "$PROJECT_DIR/Release"
echo "  removed  Debug/ and Release/ build output"

# --- 4) rename the project directory itself ----------------------------------
NEW_DIR="$(dirname "$PROJECT_DIR")/$NEW_NAME"
if [[ "$PROJECT_DIR" != "$NEW_DIR" ]]; then
  if [[ -e "$NEW_DIR" ]]; then
    echo "Error: '$NEW_DIR' already exists — refusing to overwrite." >&2
    exit 1
  fi
  mv "$PROJECT_DIR" "$NEW_DIR"
  echo "  renamed  $(basename "$PROJECT_DIR")/ -> $NEW_NAME/"
fi

cat <<EOF

Done. Next steps:
  1. Open STM32CubeIDE and import '$NEW_NAME/'
     (File > Open Projects from File System... > select the '$NEW_NAME' folder).
  2. Build. The pre-build step pulls the micro-ROS Docker image and regenerates
     the static library into micro_ros_stm32cubemx_utils/.
  3. Flash the board, then start the agent (see README.md "Run it").
EOF
