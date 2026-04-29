#!/usr/bin/env python3
"""Update .vscode/settings.json python extraPaths from install/ directory.

Finds all site-packages directories under install/ (colcon-built workspace
packages) and updates both python.autoComplete.extraPaths and
python.analysis.extraPaths in settings.json.

The pixi environment's site-packages are discovered automatically by Pylance
via the interpreter path, so they don't need to be listed here.

Usage:
    python3 .vscode/update_python_paths.py
"""

import json
from pathlib import Path
import re
import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
SETTINGS_PATH = REPO_ROOT / ".vscode" / "settings.json"
INSTALL_DIR = REPO_ROOT / "install"


def find_install_site_packages() -> list[str]:
    """Find all site-packages dirs under install/, returned as relative paths."""
    if not INSTALL_DIR.is_dir():
        print("Warning: install/ directory not found. Has the workspace been built?", file=sys.stderr)
        return []

    paths = sorted(str(p.relative_to(REPO_ROOT)) for p in INSTALL_DIR.rglob("site-packages") if p.is_dir())
    return paths


def update_settings(paths: list[str]) -> None:
    """Update the extraPaths keys in settings.json."""
    if not SETTINGS_PATH.exists():
        print(f"Error: {SETTINGS_PATH} not found", file=sys.stderr)
        sys.exit(1)

    text = SETTINGS_PATH.read_text()

    # json.loads doesn't handle trailing commas, so strip them
    cleaned = re.sub(r",\s*([}\]])", r"\1", text)
    settings = json.loads(cleaned)

    old = settings.get("python.autoComplete.extraPaths", [])

    settings["python.autoComplete.extraPaths"] = paths
    settings["python.analysis.extraPaths"] = paths

    # Write back with consistent formatting
    SETTINGS_PATH.write_text(json.dumps(settings, indent=4) + "\n")

    added = set(paths) - set(old)
    removed = set(old) - set(paths)

    print(f"Updated {SETTINGS_PATH.relative_to(REPO_ROOT)}")
    print(f"  {len(paths)} extra paths")
    if added:
        print(f"  +{len(added)} added")
    if removed:
        print(f"  -{len(removed)} removed")


def main():
    paths = find_install_site_packages()

    if not paths:
        print("Nothing to update")
        sys.exit(0)

    update_settings(paths)


if __name__ == "__main__":
    main()
