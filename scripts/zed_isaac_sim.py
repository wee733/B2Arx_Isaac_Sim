"""Pinned Stereolabs ZED Isaac Sim integration metadata and geometry.

This module deliberately contains no Isaac Sim imports so installation and
geometry checks can run in a normal Python test process.  The supported path
keeps the official sensor embedded in the core robot USD:

    my_robot.usd/Robot/b2_description/R5a/ZED_X
    -> sl.sensor.camera.ZED_Camera -> ZED SDK simulation mode
    -> zed_wrapper -> Isaac ROS Nvblox

Stereolabs does not currently provide a ZED 2/2i Isaac Sim model.  Isaac Sim
5.1 is therefore pinned to the officially supported ZED X extension v4.3.0.
"""
from __future__ import annotations

import hashlib
import math
import os
import sys
from pathlib import Path


ZED_ISAAC_SIM_VERSION = "v4.3.0"
ZED_ISAAC_SIM_RELEASE_URL = (
    "https://github.com/stereolabs/zed-isaac-sim/releases/download/v4.3.0/"
    "stereolabs-zed-isaac-sim-linux-x86_64-v4.3.0.zip"
)
ZED_ISAAC_SIM_RELEASE_SHA256 = "07e6ef3d6b667152213fc6e3ed324202bb2cbf5bc606d7551c005ebff9271486"
ZED_ISAAC_SIM_BUILD_DIRNAME = (
    "IsaacSimZED@107.3.0+main.0.a17a16d5.local.manylinux_2_35_x86_64.release"
)
ZED_EXTENSION_FILE_SHA256 = {
    "config/extension.toml": "4675b401233693245c810d2be2581583ae0d7bbc323d123b0ccf4dff41900772",
    "bin/libsl.sensor.camera.plugin.so": "270d93c2d26f2c983a30c0d151ab93c52653a502e42895743a4940bf73a6e42f",
    "data/usd/ZED_X.usdc": "aedba222a0b7db8d909f51977f35353aac15fa13e5e4153e79225c9f2e06d79b",
}

DEFAULT_ZED_ISAAC_SIM_INSTALL_ROOT = (
    Path.home() / ".local" / "share" / "zed-isaac-sim" / ZED_ISAAC_SIM_VERSION
)


def zed_install_root() -> Path:
    """Return the pinned extension install root, allowing a reproducible override."""
    override = os.environ.get("ZED_ISAAC_SIM_ROOT")
    return Path(override).expanduser().resolve() if override else DEFAULT_ZED_ISAAC_SIM_INSTALL_ROOT


def zed_build_root() -> Path:
    return zed_install_root() / ZED_ISAAC_SIM_BUILD_DIRNAME


def zed_extensions_dir() -> Path:
    return zed_build_root() / "exts"


def zed_extension_dir() -> Path:
    return zed_extensions_dir() / "sl.sensor.camera"


def zed_x_usd_path() -> Path:
    return zed_extension_dir() / "data" / "usd" / "ZED_X.usdc"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_zed_extension_dir(extension_dir: Path) -> None:
    """Verify critical files against the checksum-pinned official release."""
    missing = [
        extension_dir / relative
        for relative in ZED_EXTENSION_FILE_SHA256
        if not (extension_dir / relative).is_file()
    ]
    if missing:
        details = "\n  ".join(str(path) for path in missing)
        raise FileNotFoundError(f"Missing official Stereolabs extension files:\n  {details}")

    mismatches = []
    for relative, expected in ZED_EXTENSION_FILE_SHA256.items():
        path = extension_dir / relative
        actual = file_sha256(path)
        if actual != expected:
            mismatches.append(f"{path}: expected {expected}, got {actual}")
    if mismatches:
        raise RuntimeError(
            "Installed Stereolabs files do not match the pinned official v4.3.0 release. "
            "Reinstall with `python3 scripts/install_zed_isaac_sim.py --force`:\n  "
            + "\n  ".join(mismatches)
        )


ZED_EXTENSION_ID = "sl.sensor.camera"
ZED_EXTENSION_FULL_ID = f"{ZED_EXTENSION_ID}-{ZED_ISAAC_SIM_VERSION.removeprefix('v')}"
ZED_HELPER_NODE_TYPE = "sl.sensor.camera.ZED_Camera"
ZED_INTERNAL_STREAM_NODE_TYPE = "sl.sensor.camera.OgnZEDSimCameraNode"
_ZED_HELPER_FALLBACK_REGISTERED = False
_ZED_EXTENSION_DISABLE_HOOK = None
# The physics-sensor extension must be present when Kit starts.  Loading it
# later registers the OmniGraph node but not a functioning Isaac Sensor schema
# manager; in that state every IMU reading is invalid and the Stereolabs native
# stream node (which is gated by IsaacReadIMU) never executes.
ZED_STARTUP_EXTENSION_IDS = (
    "isaacsim.sensors.physics",
)
ZED_RUNTIME_EXTENSION_IDS = (
    "isaacsim.core.nodes",
)
ZED_RUNTIME_NODE_TYPES = (
    "isaacsim.core.nodes.IsaacReadSimulationTime",
    "isaacsim.core.nodes.IsaacReadSystemTime",
    "isaacsim.sensors.physics.IsaacReadIMU",
)
ZED_STREAM_GRAPH_PATH = "/World/StereolabsZEDStreamGraph"
ZED_X_CAMERA_MODEL = "ZED_X"

# Official Stereolabs ZED X stream settings used by the ZED SDK simulation
# path. HD1200/30 matches the Isaac ROS release-4.5 ZED X sensor config. The
# wrapper's pre-open HD1080/60 assumptions are replaced by the actual stream
# metadata after the SDK connection opens.
ZED_STREAM_RESOLUTIONS = {
    "HD1200": (1920, 1200, 741.6),
    "HD1080": (1920, 1080, 741.6),
    "SVGA": (960, 600, 370.8),
}
ZED_STREAM_FPS_BY_RESOLUTION = {
    "HD1200": (15, 30, 60),
    "HD1080": (15, 30, 60),
    "SVGA": (15, 30, 60, 120),
}
ZED_STREAM_RESOLUTION = "HD1200"
ZED_STREAM_FPS = 30
ZED_STREAM_PORT = 30000
# The Stereolabs Isaac Sim helper is the stream server and only selects its
# transport and port here. ``simulation.sim_address`` is configured on the
# zed_wrapper receiver: it points back to this Isaac Sim host, so a receiver
# running on Thor must not set it to Thor's own address.
# Use the official extension default.  BOTH exposes the network stream for a
# remote zed_wrapper while retaining the Linux IPC path for a local wrapper;
# in particular, NETWORK-only can fail to create an RTP session on a local
# receiver before the SDK has completed discovery (err:-74).
# BOTH and IPC remain selectable explicitly from the scene CLI.
ZED_STREAM_TRANSPORT = "BOTH"
ZED_STREAM_BITRATE_KBPS = 8000
ZED_STREAM_CHUNK_SIZE_BYTES = 4096
ZED_STREAM_READY_MESSAGE = "ZED Streamer initialized successfully with ID"
ZED_STREAM_INIT_ERROR_MESSAGE = "Error during zed streamer initialization"

# Isaac ROS release-4.5 ``get_zed_remappings()`` consumes these outputs from
# the official zed_wrapper.  Isaac Sim must not publish look-alike topics itself.
ZED_WRAPPER_TOPICS = {
    "depth": "/zed/zed_node/depth/depth_registered",
    "depth_info": "/zed/zed_node/depth/camera_info",
    "color": "/zed/zed_node/rgb/color/rect/image",
    "color_info": "/zed/zed_node/rgb/camera_info",
    "pose": "/zed/zed_node/pose",
}

# Measured from the official v4.3.0 ZED_X.usdc.  Both cameras look along asset
# +X; the optical centers are 0.12 m apart.
ZED_X_BASELINE_M = 0.12
ZED_X_LEFT_CAMERA_OFFSET = (0.015, +0.06, 0.015)
ZED_X_RIGHT_CAMERA_OFFSET = (0.015, -0.06, 0.015)
ZED_CORE_ROOT_REL_PATH = "b2_description/R5a/ZED_X"
ZED_X_CAMERA_PRIMS = {
    "left": "base_link/ZED_X/CameraLeft",
    "right": "base_link/ZED_X/CameraRight",
}
ZED_X_IMU_PRIM = "base_link/ZED_X/Imu_Sensor"


def add_zed_startup_kit_args(existing_kit_args: str = "") -> str:
    """Add extensions that must be loaded while Kit itself is starting."""
    tokens = existing_kit_args.split()
    enabled = {
        tokens[index + 1]
        for index, token in enumerate(tokens[:-1])
        if token == "--enable"
    }
    enabled.update(token.removeprefix("--enable=") for token in tokens if token.startswith("--enable="))
    for extension_id in ZED_STARTUP_EXTENSION_IDS:
        if extension_id not in enabled:
            tokens.extend(("--enable", extension_id))
    return " ".join(tokens)


def stream_dimensions(resolution: str = ZED_STREAM_RESOLUTION) -> tuple[int, int]:
    try:
        width, height, _ = ZED_STREAM_RESOLUTIONS[resolution]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported official ZED stream resolution {resolution!r}; "
            f"choose one of {tuple(ZED_STREAM_RESOLUTIONS)}"
        ) from exc
    return width, height


def stream_intrinsics(resolution: str = ZED_STREAM_RESOLUTION) -> tuple[float, float, float, float]:
    """Return the pinhole intrinsics authored by the official stream helper."""
    width, height, focal_px = ZED_STREAM_RESOLUTIONS[resolution]
    return focal_px, focal_px, width / 2.0, height / 2.0


def validate_zed_stream_settings(
    resolution: str,
    fps: int,
    port: int,
    transport: str,
) -> None:
    """Validate combinations accepted by the official ZED X stream helper."""
    stream_dimensions(resolution)
    allowed_fps = ZED_STREAM_FPS_BY_RESOLUTION[resolution]
    if fps not in allowed_fps:
        raise ValueError(
            f"Unsupported ZED X stream combination {resolution}@{fps}; "
            f"allowed FPS for {resolution}: {allowed_fps}"
        )
    if port <= 0 or port > 65535 or port % 2:
        raise ValueError(f"ZED stream port must be positive and even: {port}")
    if transport not in ("BOTH", "NETWORK", "IPC"):
        raise ValueError(f"Unsupported ZED transport: {transport}")


def validate_zed_configuration() -> None:
    """Validate the pinned official geometry and helper settings."""
    left = ZED_X_LEFT_CAMERA_OFFSET
    right = ZED_X_RIGHT_CAMERA_OFFSET
    baseline = math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right, strict=True)))
    if not math.isclose(baseline, ZED_X_BASELINE_M, abs_tol=1.0e-12):
        raise ValueError(f"Official ZED X baseline mismatch: {baseline} != {ZED_X_BASELINE_M}")
    validate_zed_stream_settings(
        ZED_STREAM_RESOLUTION,
        ZED_STREAM_FPS,
        ZED_STREAM_PORT,
        ZED_STREAM_TRANSPORT,
    )


def validate_zed_installation() -> None:
    """Fail with the exact reproducible installer command when the extension is absent."""
    try:
        validate_zed_extension_dir(zed_extension_dir())
    except FileNotFoundError as exc:
        raise FileNotFoundError(
            "Official Stereolabs ZED Isaac Sim extension v4.3.0 is not installed. "
            "Run `python3 scripts/install_zed_isaac_sim.py`. " + str(exc)
        ) from exc


def install_zed_stream_status_logger():
    """Remember Kit's logfile so native ZED status can be mirrored safely.

    A global ``carb.logging`` Python callback is deliberately not used here.
    Kit invokes logger callbacks synchronously from native worker threads; a
    callback that needs the Python GIL can therefore deadlock a blocking call
    such as ``SimulationContext.reset()``.  Kit already writes the required
    Stereolabs INFO line to ``/log/file``, so the scene polls that file after
    renderer updates instead.
    """
    import carb.settings

    log_file = carb.settings.get_settings().get("/log/file")
    log_path = Path(log_file).expanduser().resolve() if log_file else None
    return {
        "log_path": log_path,
        "ready": False,
        "last_error": None,
        "closed": False,
    }


def report_zed_stream_status(logger_state) -> bool:
    """Mirror one native streamer readiness/error transition to stdout."""
    if logger_state is None or logger_state.get("closed"):
        return False
    if logger_state["ready"]:
        return True

    log_path = logger_state.get("log_path")
    if log_path is None:
        return False
    try:
        with log_path.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            stream.seek(max(0, size - 256 * 1024), os.SEEK_SET)
            log_tail = stream.read().decode("utf-8", errors="replace")
    except OSError:
        return False

    plugin_lines = [
        line
        for line in log_tail.splitlines()
        if "[sl.sensor.camera.plugin]" in line
    ]
    ready_line = next(
        (line for line in reversed(plugin_lines) if ZED_STREAM_READY_MESSAGE in line),
        None,
    )
    if ready_line is not None:
        stream_id = ready_line.rsplit("ID", 1)[-1].strip()
        logger_state["ready"] = True
        logger_state["last_error"] = None
        print(
            "[READY]: Official Stereolabs ZED Streamer initialized "
            f"successfully with ID {stream_id}. It is now safe to start zed_wrapper.",
            flush=True,
        )
        return True

    error_line = next(
        (line for line in reversed(plugin_lines) if ZED_STREAM_INIT_ERROR_MESSAGE in line),
        None,
    )
    if error_line is not None and error_line != logger_state["last_error"]:
        logger_state["last_error"] = error_line
        message = error_line.split("[sl.sensor.camera.plugin]", 1)[-1].strip()
        print(
            f"[ERROR]: Official Stereolabs streamer initialization failed: {message}",
            flush=True,
        )
    return False


def remove_zed_stream_status_logger(logger_state) -> None:
    """Stop polling state returned by :func:`install_zed_stream_status_logger`."""
    if logger_state is None:
        return
    logger_state["closed"] = True


def enable_zed_extension(*, max_app_updates: int = 32) -> str:
    """Register and enable the pinned third-party extension in a running Kit app.

    The extension path points directly at ``sl.sensor.camera``.  Using a normal
    collection path makes Kit discover it asynchronously and can race creation
    of the ZED Camera Helper node in short/headless runs.
    """
    global _ZED_EXTENSION_DISABLE_HOOK, _ZED_HELPER_FALLBACK_REGISTERED

    validate_zed_installation()
    if max_app_updates <= 0:
        raise ValueError(f"max_app_updates must be positive: {max_app_updates}")

    import omni.ext
    import omni.graph.core as og
    import omni.kit.app

    app = omni.kit.app.get_app()
    extension_manager = app.get_extension_manager()
    extension_path = str(zed_extension_dir())

    missing_startup_extensions = [
        extension_id
        for extension_id in ZED_STARTUP_EXTENSION_IDS
        if not extension_manager.is_extension_enabled(extension_id)
    ]
    if missing_startup_extensions:
        startup_args = " ".join(
            f"--enable {extension_id}" for extension_id in missing_startup_extensions
        )
        raise RuntimeError(
            "The official ZED IMU prerequisites must be enabled while Kit starts, "
            "not dynamically after SimulationApp creation. Add these AppLauncher kit args: "
            f"{startup_args}"
        )

    def verified_enabled_extension_id() -> str:
        extension_id = extension_manager.get_enabled_extension_id(ZED_EXTENSION_ID)
        if not extension_id:
            return ""
        active_path = Path(extension_manager.get_extension_path(extension_id)).resolve()
        expected_path = zed_extension_dir().resolve()
        if extension_id != ZED_EXTENSION_FULL_ID or active_path != expected_path:
            raise RuntimeError(
                "A different Stereolabs extension is already active; refusing to claim the pinned official "
                f"release is in use. active={extension_id!r} at {active_path}, "
                f"expected={ZED_EXTENSION_FULL_ID!r} at {expected_path}"
            )
        return extension_id

    # Refuse a pre-enabled, unpinned copy before changing any other Kit state.
    enabled_extension_id = verified_enabled_extension_id()

    # The Stereolabs helper creates this official Isaac Sim node internally but
    # its v4.3.0 manifest does not declare the owning extension.  It is normally
    # already pulled in by the startup physics-sensor dependency; retain this
    # guarded enable for non-Isaac-Lab experiences.
    for runtime_extension_id in ZED_RUNTIME_EXTENSION_IDS:
        if extension_manager.is_extension_enabled(runtime_extension_id):
            continue
        if not extension_manager.set_extension_enabled_immediate(runtime_extension_id, True):
            raise RuntimeError(f"Failed to enable ZED runtime extension {runtime_extension_id!r}")
    app.update()

    if not enabled_extension_id:
        extension_manager.add_path(extension_path, omni.ext.ExtensionPathType.DIRECT_PATH)
        for _ in range(max_app_updates):
            app.update()
            if verified_enabled_extension_id():
                break
            if extension_manager.set_extension_enabled_immediate(ZED_EXTENSION_FULL_ID, True):
                break
        else:
            raise RuntimeError(
                f"Kit did not discover/enable {ZED_EXTENSION_ID!r} from the pinned direct path "
                f"after {max_app_updates} updates: {extension_path}"
            )

    extension_id = ""
    registered_nodes: set[str] = set()
    for _ in range(max_app_updates):
        app.update()
        extension_id = verified_enabled_extension_id()
        registered_nodes = set(og.get_registered_nodes())
        if (
            extension_id
            and ZED_HELPER_NODE_TYPE in registered_nodes
            and ZED_INTERNAL_STREAM_NODE_TYPE in registered_nodes
            and all(node_type in registered_nodes for node_type in ZED_RUNTIME_NODE_TYPES)
        ):
            return extension_id
        if extension_id and ZED_INTERNAL_STREAM_NODE_TYPE in registered_nodes:
            break

    if ZED_INTERNAL_STREAM_NODE_TYPE not in registered_nodes:
        registered_zed_nodes = sorted(
            node_type for node_type in registered_nodes if node_type.startswith(f"{ZED_EXTENSION_ID}.")
        )
        raise RuntimeError(
            f"Enabled extension {extension_id or ZED_EXTENSION_ID!r}, but its native OmniGraph node "
            f"{ZED_INTERNAL_STREAM_NODE_TYPE!r} was not registered; "
            f"registered Stereolabs nodes: {registered_zed_nodes}"
        )

    # In Isaac Sim 5.1 headless tests, dynamically enabling the official v4.3.0
    # thin release registers its native stream node but not always its Python
    # helper.  Register the packaged, generated database against the packaged
    # implementation exactly as OmniGraph's normal extension scanner does; no
    # node or algorithm is reimplemented here.
    if ZED_HELPER_NODE_TYPE not in registered_nodes:
        try:
            from sl.sensor.camera.nodes.SlCameraStreamer import SlCameraStreamer
            from sl.sensor.camera.ogn.SlCameraStreamerDatabase import SlCameraStreamerDatabase

            expected_root = zed_extension_dir().resolve()
            imported_files = (
                Path(sys.modules[SlCameraStreamer.__module__].__file__).resolve(),
                Path(sys.modules[SlCameraStreamerDatabase.__module__].__file__).resolve(),
            )
            unexpected_files = [path for path in imported_files if not path.is_relative_to(expected_root)]
            if unexpected_files:
                raise RuntimeError(
                    f"Refusing stale/unpinned Stereolabs helper modules: {unexpected_files}; "
                    f"expected files below {expected_root}"
                )
            SlCameraStreamerDatabase.register(SlCameraStreamer)
            _ZED_HELPER_FALLBACK_REGISTERED = True
            if _ZED_EXTENSION_DISABLE_HOOK is None:
                def deregister_owned_helper(_extension_id: str) -> None:
                    global _ZED_HELPER_FALLBACK_REGISTERED
                    if not _ZED_HELPER_FALLBACK_REGISTERED:
                        return
                    SlCameraStreamerDatabase.deregister()
                    _ZED_HELPER_FALLBACK_REGISTERED = False

                _ZED_EXTENSION_DISABLE_HOOK = extension_manager.subscribe_to_extension_enable(
                    on_enable_fn=lambda _extension_id: None,
                    on_disable_fn=deregister_owned_helper,
                    ext_name=ZED_EXTENSION_ID,
                    hook_name="b2arx official ZED helper fallback lifecycle",
                )
        except Exception as exc:
            raise RuntimeError(
                f"Failed to register the official packaged helper {ZED_HELPER_NODE_TYPE!r}"
            ) from exc

    app.update()
    registered_nodes = set(og.get_registered_nodes())
    extension_id = verified_enabled_extension_id()
    missing_runtime_nodes = sorted(set(ZED_RUNTIME_NODE_TYPES) - registered_nodes)
    if extension_id and ZED_HELPER_NODE_TYPE in registered_nodes and not missing_runtime_nodes:
        return extension_id

    if missing_runtime_nodes:
        raise RuntimeError(
            "Official Stereolabs helper prerequisites were enabled but their OmniGraph nodes "
            f"were not registered: {missing_runtime_nodes}"
        )

    registered_zed_nodes = sorted(
        node_type for node_type in registered_nodes if node_type.startswith(f"{ZED_EXTENSION_ID}.")
    )
    raise RuntimeError(
        f"Enabled extension {extension_id or ZED_EXTENSION_ID!r}, but OmniGraph node "
        f"{ZED_HELPER_NODE_TYPE!r} was not registered after {max_app_updates} updates; "
        f"registered Stereolabs nodes: {registered_zed_nodes}"
    )


def setup_zed_stream_graph(
    camera_prim_path: str,
    *,
    resolution: str = ZED_STREAM_RESOLUTION,
    fps: int = ZED_STREAM_FPS,
    port: int = ZED_STREAM_PORT,
    transport: str = ZED_STREAM_TRANSPORT,
) -> str:
    """Create the official Stereolabs ZED Camera Helper action graph."""
    validate_zed_stream_settings(resolution, fps, port, transport)

    import omni.graph.core as og

    og.Controller.edit(
        {"graph_path": ZED_STREAM_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ZEDCameraHelper", ZED_HELPER_NODE_TYPE),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "ZEDCameraHelper.inputs:execIn"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ZEDCameraHelper.inputs:cameraPrim", [camera_prim_path]),
                ("ZEDCameraHelper.inputs:cameraModel", ZED_X_CAMERA_MODEL),
                ("ZEDCameraHelper.inputs:resolution", resolution),
                ("ZEDCameraHelper.inputs:fps", int(fps)),
                ("ZEDCameraHelper.inputs:streamingPort", int(port)),
                ("ZEDCameraHelper.inputs:transportLayerMode", transport),
                ("ZEDCameraHelper.inputs:bitrate", ZED_STREAM_BITRATE_KBPS),
                ("ZEDCameraHelper.inputs:chunkSize", ZED_STREAM_CHUNK_SIZE_BYTES),
            ],
        },
    )
    return ZED_STREAM_GRAPH_PATH
