from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Inspect the official RealSense D455 USD prim tree.")
parser.add_argument(
    "--usd_path",
    default="",
    help="D455 USD path or URL to inspect. Defaults to Isaac's official asset root.",
)
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric API and use USD instead.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from d455_geometry import D455_OFFICIAL_CAMERA_PRIMS, resolve_d455_usd_path


def _xform_summary(prim):
    """Return (local_translation, world_translation, world_look_dir, world_up_dir, world_quat_wxyz)."""
    from pxr import Gf, Usd, UsdGeom

    xformable = UsdGeom.Xformable(prim)
    local = xformable.GetLocalTransformation(Usd.TimeCode.Default())
    world = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    local_t = local.ExtractTranslation()
    world_t = world.ExtractTranslation()
    # USD cameras look down local -Z (OpenGL); up is local +Y.
    look = world.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
    up = world.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0))
    quat = world.ExtractRotationQuat()
    imag = quat.GetImaginary()
    quat_wxyz = (quat.GetReal(), imag[0], imag[1], imag[2])
    return (
        np.array(local_t, dtype=np.float64),
        np.array(world_t, dtype=np.float64),
        np.array(look, dtype=np.float64),
        np.array(up, dtype=np.float64),
        np.array(quat_wxyz, dtype=np.float64),
    )


def main() -> None:
    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 60.0, device=args_cli.device, use_fabric=not args_cli.disable_fabric)
    sim = SimulationContext(sim_cfg)

    usd_path = args_cli.usd_path or resolve_d455_usd_path(ISAAC_NUCLEUS_DIR)
    cfg = sim_utils.UsdFileCfg(usd_path=usd_path)
    cfg.func("/World/D455", cfg)
    sim.reset()

    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    print(f"[D455_INSPECT]: usd_path={usd_path}", flush=True)

    # 1) List every Camera/stream-like prim path.
    camera_prims = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not path.startswith("/World/D455"):
            continue
        type_name = prim.GetTypeName()
        is_camera = type_name == "Camera"
        if is_camera or "camera" in path.lower() or "stream" in path.lower() or "imu" in path.lower():
            print(f"[D455_INSPECT]: {path} type={type_name}", flush=True)
        if is_camera:
            camera_prims.append(prim)

    # 2) Dump transform of each official named camera prim and report offsets vs the color camera.
    np.set_printoptions(precision=4, suppress=True)
    named_world = {}
    print("[D455_XFORM]: --- per-camera transforms (asset mounted at /World/D455 origin) ---", flush=True)
    for stream, prim_name in D455_OFFICIAL_CAMERA_PRIMS.items():
        matches = [p for p in camera_prims if p.GetPath().pathString.endswith("/" + prim_name)]
        if not matches:
            print(f"[D455_XFORM]: {stream:6s} prim '{prim_name}' NOT FOUND", flush=True)
            continue
        prim = matches[0]
        local_t, world_t, look, up, quat = _xform_summary(prim)
        named_world[stream] = world_t
        print(
            f"[D455_XFORM]: {stream:6s} {prim.GetPath().pathString}\n"
            f"             local_t ={local_t}\n"
            f"             world_t ={world_t}\n"
            f"             look_dir={look}  up_dir={up}\n"
            f"             quat_wxyz={quat}",
            flush=True,
        )

    if "color" in named_world:
        color_t = named_world["color"]
        print("[D455_DELTA]: --- world translation of each stream relative to COLOR ---", flush=True)
        for stream, world_t in named_world.items():
            d = world_t - color_t
            print(
                f"[D455_DELTA]: {stream:6s} dx={d[0]:+.4f} dy={d[1]:+.4f} dz={d[2]:+.4f}  "
                f"(|d|={np.linalg.norm(d):.4f} m)",
                flush=True,
            )


if __name__ == "__main__":
    main()
    simulation_app.close()
