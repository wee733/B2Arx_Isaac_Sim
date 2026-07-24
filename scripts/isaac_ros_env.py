"""Process-start environment required by Isaac Sim's bundled ROS 2 bridge.

Isaac Sim's ROS 2 extension loads ``librmw_implementation.so`` with ``dlopen``.
Its dependent Jazzy libraries live beside it, but that directory is not added
to the dynamic-loader search path by pip-based Isaac Sim/Isaac Lab launches.
Because glibc snapshots ``LD_LIBRARY_PATH`` when the process starts, changing
the variable after Kit has launched is too late.  Re-exec once, before any
Isaac imports, so the normal one-command scene launcher works reliably.
"""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys
from typing import Mapping, Sequence


ROS_BRIDGE_REQUEST_FLAGS = frozenset(("--ros2", "--nav2"))
ROS_BRIDGE_RELATIVE_LIB_DIR = Path("exts") / "isaacsim.ros2.bridge" / "jazzy" / "lib"
ROS_BRIDGE_REQUIRED_LIBRARIES = ("librmw_implementation.so", "libament_index_cpp.so")


def ros_bridge_requested(argv: Sequence[str]) -> bool:
    """Return whether this scene invocation needs the Isaac ROS 2 bridge."""
    return any(argument in ROS_BRIDGE_REQUEST_FLAGS for argument in argv)


def find_bundled_ros_bridge_lib_dir() -> Path:
    """Locate and validate Isaac Sim's bundled Jazzy shared-library directory."""
    spec = importlib.util.find_spec("isaacsim")
    if spec is None:
        raise RuntimeError(
            "Isaac Sim is not importable. Activate the isaaclab conda environment before "
            "launching the B2ARX scene."
        )

    package_roots: list[Path] = []
    if spec.submodule_search_locations:
        package_roots.extend(Path(path).resolve() for path in spec.submodule_search_locations)
    if spec.origin:
        package_roots.append(Path(spec.origin).resolve().parent)

    for package_root in dict.fromkeys(package_roots):
        library_dir = package_root / ROS_BRIDGE_RELATIVE_LIB_DIR
        if all((library_dir / library).is_file() for library in ROS_BRIDGE_REQUIRED_LIBRARIES):
            return library_dir

    searched = ", ".join(str(root / ROS_BRIDGE_RELATIVE_LIB_DIR) for root in package_roots)
    raise RuntimeError(
        "Isaac Sim's bundled Jazzy ROS 2 bridge libraries are incomplete or missing. "
        f"Searched: {searched or '<no Isaac Sim package roots>'}"
    )


def _path_entries(value: str) -> list[str]:
    return [entry for entry in value.split(os.pathsep) if entry]


def _contains_path(value: str, expected: Path) -> bool:
    expected_resolved = expected.resolve()
    for entry in _path_entries(value):
        try:
            if Path(entry).expanduser().resolve() == expected_resolved:
                return True
        except OSError:
            continue
    return False


def build_ros_bridge_environment(
    library_dir: Path,
    environment: Mapping[str, str],
) -> dict[str, str]:
    """Return an environment with the bundled Jazzy bridge directory first."""
    updated = dict(environment)
    current_library_path = updated.get("LD_LIBRARY_PATH", "")
    entries = [str(library_dir.resolve())]
    entries.extend(
        entry
        for entry in _path_entries(current_library_path)
        if not _contains_path(entry, library_dir)
    )
    updated["LD_LIBRARY_PATH"] = os.pathsep.join(entries)
    updated.setdefault("ROS_DISTRO", "jazzy")
    return updated


def ensure_ros_bridge_process_environment(argv: Sequence[str] | None = None) -> None:
    """Re-exec once when a ROS-enabled scene lacks the bridge loader path."""
    arguments = tuple(sys.argv[1:] if argv is None else argv)
    if not ros_bridge_requested(arguments):
        return

    library_dir = find_bundled_ros_bridge_lib_dir()
    if _contains_path(os.environ.get("LD_LIBRARY_PATH", ""), library_dir):
        os.environ.setdefault("ROS_DISTRO", "jazzy")
        return

    updated_environment = build_ros_bridge_environment(library_dir, os.environ)
    os.execvpe(
        sys.executable,
        [sys.executable, *sys.argv],
        updated_environment,
    )
    raise RuntimeError("Re-executing with the Isaac ROS 2 bridge environment unexpectedly returned")

