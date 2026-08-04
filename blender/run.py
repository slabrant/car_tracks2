"""Entry point. docs/SPEC.md §8.

    blender --background --python blender/run.py -- --part straight
    blender --background --python blender/run.py -- --part curve --radius 100
    blender --background --python blender/run.py -- --all

Geometry is computed and validated in trackcore before Blender sees any of it.
Blender builds the mesh and writes the file; if the mesh is invalid it never
gets that far.
"""

from __future__ import annotations

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from blender.build import from_object, reset_scene, to_object  # noqa: E402
from blender.export import export  # noqa: E402
from parts import CATALOGUE  # noqa: E402
from trackcore import DEFAULT, check, expected_volume, sweep  # noqa: E402


def build_part(name: str, collection, **kwargs):
    if name not in CATALOGUE:
        raise SystemExit(f"unknown part {name!r}; have {sorted(CATALOGUE)}")
    path = CATALOGUE[name](**kwargs)
    mesh_data = sweep(path, DEFAULT)

    stats = check(mesh_data, name=name)
    ideal = expected_volume(path, DEFAULT)
    error = abs(stats["volume_mm3"] - ideal) / ideal

    print(f"  {name:10s} len={path.length:7.2f} mm  "
          f"V={int(stats['verts']):5d} F={int(stats['faces']):5d}  "
          f"vol={stats['volume_mm3']:9.2f} mm3  "
          f"({error * 100:.4f}% under the ideal {ideal:.2f})")

    return to_object(mesh_data, name, collection), path


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a track part")
    parser.add_argument("--part", default="straight", choices=sorted(CATALOGUE))
    parser.add_argument("--all", action="store_true", help="build every part")
    parser.add_argument("--outdir", default="out")
    parser.add_argument("--format", default="stl", choices=["stl", "3mf"])
    parser.add_argument("--length", type=float, default=None)
    parser.add_argument("--radius", type=float, default=None)
    parser.add_argument("--angle", type=float, default=None, help="degrees")
    parser.add_argument("--bank", type=float, default=None, help="degrees")
    parser.add_argument("--run", type=float, default=None)
    parser.add_argument("--rise", type=float, default=None)
    return parser.parse_args(argv)


def kwargs_for(name: str, args: argparse.Namespace) -> dict:
    mapping = {
        "straight": {"length": args.length},
        "curve": {"radius": args.radius, "angle_deg": args.angle,
                  "bank_deg": args.bank},
        "ramp": {"run": args.run, "rise": args.rise},
        "s_bend": {"radius": args.radius, "angle_deg": args.angle},
    }
    return {k: v for k, v in mapping[name].items() if v is not None}


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parse_args(argv)

    collection = reset_scene()
    names = sorted(CATALOGUE) if args.all else [args.part]

    print()
    print(f"output : {os.path.abspath(args.outdir)}")
    print()

    for name in names:
        obj, _path = build_part(name, collection, **kwargs_for(name, args))
        out = os.path.join(args.outdir, f"{name}.{args.format}")
        export([obj], out)

        # re-check what Blender actually holds, not only what trackcore made
        check(from_object(obj), name=f"{name} (after Blender)")

    print()
    print(f"{len(names)} part(s) built, validated and written.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
