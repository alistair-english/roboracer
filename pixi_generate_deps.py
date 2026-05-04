#!/usr/bin/env python3
"""Parse all package.xml files in src/ and update pixi.toml with dependencies.

Re-generate with: task update_deps
"""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

REPO_ROOT = Path(__file__).resolve().parent
SRC_DIR = REPO_ROOT / "src"
PIXI_TOML = REPO_ROOT / "pixi.toml"

# Maps for dependencies that don't follow the standard ROS naming convention.
# Values are conda-forge package names.
SYSTEM_DEP_MAP = {
    "libpcl-all": "pcl",
    "libpcl-all-dev": "pcl",
    "libopencv-dev": "opencv",
    "eigen": "eigen",
    "cgal": "cgal-cpp",
    "libusb-1.0-dev": "libusb",
    "tbb": "tbb",
    "robin-map-dev": None,  # provided via custom robin package
    "apr": None,  # not needed at build time
    # GStreamer
    "gstreamer1.0": "gstreamer",
    "gstreamer1.0-plugins-base": "gst-plugins-base",
    "gstreamer1.0-plugins-good": "gst-plugins-good",
    "gstreamer1.0-plugins-ugly": "gst-plugins-ugly",
    "gstreamer1.0-libav": "gst-libav",
    "libgstreamer-plugins-base1.0-dev": "gst-plugins-base",
    "libgstreamer-plugins-bad1.0-dev": "gst-plugins-bad",
    "libgstreamer1.0-dev": "gstreamer",
    "libsoup2-dev": None,  # EOL
    "libjson-glib": "json-glib",
    "libexpected-dev": None,  # provided via ros-humble-tl-expected
    "fmt": "fmt",
}

# Dependencies provided by the system or devshell, not ROS packages
IGNORED_DEPS = {
    "git",
    "pkg-config",
    "python3",  # provided by the pixi env's python
}

PYTHON_DEP_MAP = {
    "python3-matplotlib": "matplotlib",
    "python3-numpy": "numpy",
    "python3-pil": "pillow",
    "python3-pyqt5": "pyqt",
    "python3-click": "click",
    "python3-flask": "flask",
    "python3-jinja2": "jinja2",
    "python3-jsonschema": "jsonschema",
    "python3-yaml": "pyyaml",
    "python3-transforms3d": "transforms3d",
    "python3-typeguard": "typeguard",
}

# Package names from VCS-imported repos (dependencies.repos).
# Used when src/deps/ hasn't been cloned yet.
VCS_REPO_NAMES = {
    "clingwrap",
    "jig",
    "breadcrumb",
    "ublox_dgnss",
    "livox_ros_driver2",
    "kiss_icp",
    "gst_pipeline",
    "ros_gst_bridge",
}

DEP_TAGS = {"depend", "build_depend", "exec_depend", "buildtool_depend"}

# Transitive dependencies that need to be explicitly included.
# When a package depends on any key, the corresponding values are also included.
IMPLICIT_TRANSITIVE_DEPS: dict[str, set[str]] = {
    "ament_cmake": {"ament_cmake_core"},
    "ament_cmake_auto": {"ament_cmake_core"},
}

# ROS deps that are not available on robostack-humble or not actually used.
DROPPED_ROS_DEPS = {
    "sophus",
}


def discover_vcs_package_names() -> set[str]:
    """Discover actual package names from ros/deps/ if it exists."""
    deps_dir = SRC_DIR / "deps"
    if not deps_dir.is_dir():
        return VCS_REPO_NAMES

    names = set()
    for pkg_xml in deps_dir.rglob("package.xml"):
        try:
            tree = ET.parse(pkg_xml)
            name_el = tree.getroot().find("name")
            if name_el is not None and name_el.text:
                names.add(name_el.text.strip())
        except ET.ParseError:
            continue
    return names if names else VCS_REPO_NAMES


def parse_packages() -> tuple[set[str], set[str]]:
    """Return (internal_names, all external deps)."""
    internal_names: set[str] = set()
    all_deps: set[str] = set()
    catkin_packages: set[str] = set()

    for pkg_xml in SRC_DIR.rglob("package.xml"):
        try:
            tree = ET.parse(pkg_xml)
        except ET.ParseError as e:
            print(f"WARNING: Failed to parse {pkg_xml}: {e}", file=sys.stderr)
            continue

        root = tree.getroot()
        name_el = root.find("name")
        if name_el is None or not name_el.text:
            continue
        pkg_name = name_el.text.strip()

        # Check if catkin package
        buildtool_deps = {el.text.strip() for el in root.findall("buildtool_depend") if el.text}
        if "catkin" in buildtool_deps:
            catkin_packages.add(pkg_name)
            continue

        internal_names.add(pkg_name)

        # Collect all dependencies (including from VCS packages, whose
        # transitive deps still need to come from the pixi environment)
        for tag in DEP_TAGS:
            for el in root.findall(tag):
                if el.text:
                    all_deps.add(el.text.strip())

    # Also add VCS repo names that may not be cloned yet
    vcs_names = discover_vcs_package_names()
    internal_names |= vcs_names

    # Expand implicit transitive dependencies
    for dep in list(all_deps):
        if dep in IMPLICIT_TRANSITIVE_DEPS:
            all_deps |= IMPLICIT_TRANSITIVE_DEPS[dep]

    # Remove internal deps
    external_deps = all_deps - internal_names
    # Also remove catkin packages and catkin itself
    external_deps -= catkin_packages
    external_deps.discard("catkin")

    return internal_names, external_deps


def classify_deps(deps: set[str]) -> tuple[list[str], list[str], list[str]]:
    """Classify deps into (ros_pkgs, system_pkgs, python_pkgs) as conda-forge names."""
    ros_pkgs: list[str] = []
    system_pkgs: list[str] = []
    python_pkgs: list[str] = []

    for dep in deps:
        if dep in IGNORED_DEPS:
            continue
        elif dep in PYTHON_DEP_MAP:
            python_pkgs.append(PYTHON_DEP_MAP[dep])
        elif dep in SYSTEM_DEP_MAP:
            conda_name = SYSTEM_DEP_MAP[dep]
            if conda_name is not None:
                system_pkgs.append(conda_name)
        else:
            # ROS package: underscore -> hyphen, prefixed with ros-humble-
            ros_name = dep.replace("_", "-")
            if ros_name in DROPPED_ROS_DEPS:
                continue
            ros_pkgs.append(f"ros-humble-{ros_name}")

    ros_pkgs = sorted(set(ros_pkgs))
    system_pkgs = sorted(set(system_pkgs))
    python_pkgs = sorted(set(python_pkgs))
    return ros_pkgs, system_pkgs, python_pkgs


def update_pixi_toml(ros_pkgs: list[str], system_pkgs: list[str], python_pkgs: list[str]) -> None:
    """Update pixi.toml by replacing all package-driven deps between marker comments."""
    content = PIXI_TOML.read_text()

    begin_marker = "# --- BEGIN AUTO-GENERATED DEPS (do not edit manually) ---"
    end_marker = "# --- END AUTO-GENERATED DEPS ---"

    begin_idx = content.find(begin_marker)
    end_idx = content.find(end_marker)

    if begin_idx == -1 or end_idx == -1:
        print(
            f"ERROR: Could not find DEPS marker comments in {PIXI_TOML}",
            file=sys.stderr,
        )
        sys.exit(1)

    lines = [begin_marker, "# Generated by: python3 pixi_generate_deps.py\n"]

    if system_pkgs:
        lines.append("# System libraries")
        lines.extend(f'{pkg} = "*"' for pkg in system_pkgs)
        lines.append("")

    if python_pkgs:
        lines.append("# Python")
        lines.extend(f'{pkg} = "*"' for pkg in python_pkgs)
        lines.append("")

    if ros_pkgs:
        lines.append("# ROS packages")
        lines.extend(f'{pkg} = "*"' for pkg in ros_pkgs)
        lines.append("")

    replacement = "\n".join(lines)
    content = content[:begin_idx] + replacement + content[end_idx:]

    PIXI_TOML.write_text(content)
    print(f"Updated {PIXI_TOML}")
    print(f"  {len(system_pkgs)} system, {len(python_pkgs)} python, {len(ros_pkgs)} ROS packages")


def main() -> None:
    internal_names, external_deps = parse_packages()
    ros_pkgs, system_pkgs, python_pkgs = classify_deps(external_deps)

    print(f"Found {len(internal_names)} internal packages")
    print(f"Found {len(external_deps)} external dependencies")

    update_pixi_toml(ros_pkgs, system_pkgs, python_pkgs)


if __name__ == "__main__":
    main()
