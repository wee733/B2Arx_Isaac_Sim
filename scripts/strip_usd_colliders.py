from __future__ import annotations

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Inspect or remove collider APIs from a USD asset.")
parser.add_argument("usd_path", type=str, help="USD file to edit in place.")
parser.add_argument("--check", action="store_true", help="Only report collider API count without editing the USD.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics  # noqa: E402
from omni.physx.scripts import utils as physx_utils  # noqa: E402


def main() -> None:
    usd_path = Path(args_cli.usd_path).expanduser().resolve()
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {usd_path}")

    collider_prims = [
        prim
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.MeshCollisionAPI)
    ]
    if args_cli.check:
        print(f"[INFO]: Collider API prim count: {len(collider_prims)} in {usd_path}", flush=True)
        return

    removed_count = 0
    for prim in collider_prims:
        physx_utils.removeCollider(prim)
        removed_count += 1

    stage.Save()
    print(f"[INFO]: Removed collider APIs from {removed_count} prim(s): {usd_path}", flush=True)


if __name__ == "__main__":
    main()
    simulation_app.close()
