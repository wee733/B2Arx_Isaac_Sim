#!/usr/bin/env python3
"""Install the pinned official Stereolabs extension for Isaac Sim 5.1.

The release URL and SHA-256 are taken from the official Stereolabs GitHub
release v4.3.0, whose release notes explicitly target Isaac Sim 5.1.
"""
from __future__ import annotations

import argparse
import shutil
import tempfile
import urllib.request
import zipfile
from pathlib import Path

try:
    from .zed_isaac_sim import (
        ZED_ISAAC_SIM_BUILD_DIRNAME,
        ZED_ISAAC_SIM_RELEASE_SHA256,
        ZED_ISAAC_SIM_RELEASE_URL,
        ZED_ISAAC_SIM_VERSION,
        file_sha256,
        validate_zed_extension_dir,
        zed_install_root,
    )
except ImportError:  # direct execution: python3 scripts/install_zed_isaac_sim.py
    from zed_isaac_sim import (
        ZED_ISAAC_SIM_BUILD_DIRNAME,
        ZED_ISAAC_SIM_RELEASE_SHA256,
        ZED_ISAAC_SIM_RELEASE_URL,
        ZED_ISAAC_SIM_VERSION,
        file_sha256,
        validate_zed_extension_dir,
        zed_install_root,
    )


def install(destination: Path, force: bool = False) -> Path:
    expected_extension = destination / ZED_ISAAC_SIM_BUILD_DIRNAME / "exts" / "sl.sensor.camera"
    if expected_extension.is_dir() and not force:
        validate_zed_extension_dir(expected_extension)
        print(f"[OK] Stereolabs ZED Isaac Sim {ZED_ISAAC_SIM_VERSION} already installed: {expected_extension}")
        return expected_extension

    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="zed-isaac-sim-") as temp_dir_name:
        temp_dir = Path(temp_dir_name)
        archive = temp_dir / "zed-isaac-sim.zip"
        extracted = temp_dir / "extracted"
        print(f"[INFO] Downloading official release: {ZED_ISAAC_SIM_RELEASE_URL}")
        urllib.request.urlretrieve(ZED_ISAAC_SIM_RELEASE_URL, archive)
        actual_sha256 = file_sha256(archive)
        if actual_sha256 != ZED_ISAAC_SIM_RELEASE_SHA256:
            raise RuntimeError(
                "Stereolabs release checksum mismatch: "
                f"expected {ZED_ISAAC_SIM_RELEASE_SHA256}, got {actual_sha256}"
            )
        with zipfile.ZipFile(archive) as zip_file:
            zip_file.extractall(extracted)
        source_build = extracted / ZED_ISAAC_SIM_BUILD_DIRNAME
        if not source_build.is_dir():
            raise RuntimeError(f"Official release layout changed; missing {source_build}")
        if destination.exists():
            if not force:
                raise FileExistsError(f"Install destination already exists: {destination}")
            shutil.rmtree(destination)
        destination.mkdir(parents=True)
        shutil.move(str(source_build), str(destination / ZED_ISAAC_SIM_BUILD_DIRNAME))

    if not expected_extension.is_dir():
        raise RuntimeError(f"Installation finished but extension is missing: {expected_extension}")
    validate_zed_extension_dir(expected_extension)
    print(f"[OK] Installed official Stereolabs extension: {expected_extension}")
    return expected_extension


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--destination", type=Path, default=zed_install_root())
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    install(args.destination.expanduser().resolve(), force=args.force)


if __name__ == "__main__":
    main()
