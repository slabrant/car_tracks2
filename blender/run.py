"""Entry point. docs/SPEC.md §8.

    blender --background --python blender/run.py -- --part straight
    blender --background --python blender/run.py -- --part x_rounded
    blender --background --python blender/run.py -- --all

Geometry is computed and validated in trackcore before Blender sees any of it.
Blender builds the mesh, unions the junction slabs, and writes the file. A
junction is re-validated after the union, per §5.3; if it fails it is never
exported and this exits non-zero.
"""

from __future__ import annotations

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from blender.boolean import apply_boolean  # noqa: E402
from blender.build import from_object, reset_scene, to_object  # noqa: E402
from blender.cleanup import clean  # noqa: E402
from blender.export import export  # noqa: E402
from parts import CATALOGUE, build  # noqa: E402
from trackcore import DEFAULT, check  # noqa: E402


def build_part(name: str, collection, connectors: bool = True, **kwargs):
    piece = build(name, DEFAULT, connectors=connectors, **kwargs)

    # every input must be a valid solid in its own right before any boolean;
    # otherwise a failure afterwards would wrongly blame the solver
    for index, solid in enumerate(piece.every_solid()):
        check(solid, name=f"{name} input {index}")

    target = to_object(piece.solids[0], name, collection)
    for operation, meshes in piece.stages():
        for index, mesh in enumerate(meshes):
            tool = to_object(mesh, f"{name}_{operation}_{index}", collection)
            apply_boolean(target, tool, operation)

    if piece.needs_boolean:
        clean(target)
    stats = check(from_object(target), name=f"{name} (after booleans)")
    note = (f"{len(piece.solids)}+{len(piece.cuts)}-{len(piece.additions)}+"
            if piece.needs_boolean else "swept")

    print(f"  {name:12s} {note:14s} V={int(stats['verts']):5d} "
          f"F={int(stats['faces']):5d}  vol={stats['volume_mm3']:9.2f} mm3")
    return target


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a track part")
    parser.add_argument("--part", default="straight_full",
                        choices=sorted(set(CATALOGUE) | {"s_bend"}))
    parser.add_argument("--all", action="store_true", help="build every part")
    parser.add_argument("--outdir", default="out")
    parser.add_argument("--format", default="stl", choices=["stl", "3mf"])
    parser.add_argument("--length", type=float, default=None)
    parser.add_argument("--radius", type=float, default=None)
    parser.add_argument("--angle", type=float, default=None, help="degrees")
    parser.add_argument("--bank", type=float, default=None, help="degrees")
    parser.add_argument("--run", type=float, default=None)
    parser.add_argument("--rise", type=float, default=None)
    parser.add_argument("--corner-radius", type=float, default=None)
    parser.add_argument("--drift", type=float, default=None,
                        help="how far a loop steps sideways, mm")
    parser.add_argument("--no-connectors", action="store_true",
                        help="flat-ended pieces, for measuring the bare body")
    return parser.parse_args(argv)


def kwargs_for(name: str, args: argparse.Namespace) -> dict:
    """Overrides by part family, so the catalogue can grow without edits here."""
    if name.startswith("straight"):
        chosen = {"length": args.length}
    elif name.startswith("curve") or name == "s_bend":
        chosen = {"radius": args.radius, "angle_deg": args.angle,
                  "bank_deg": args.bank}
    elif name.startswith("ramp"):
        chosen = {"run": args.run, "rise": args.rise}
    elif name.startswith("loop"):
        chosen = {"radius": args.radius, "drift": args.drift}
    else:
        chosen = {"corner_radius": args.corner_radius}
    if name == "s_bend":
        chosen.pop("bank_deg", None)
    return {k: v for k, v in chosen.items() if v is not None}


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parse_args(argv)

    names = CATALOGUE if args.all else [args.part]

    print()
    print(f"output : {os.path.abspath(args.outdir)}")
    print()

    for name in names:
        collection = reset_scene()
        obj = build_part(name, collection,
                         connectors=not args.no_connectors,
                         **kwargs_for(name, args))
        export([obj], os.path.join(args.outdir, f"{name}.{args.format}"))

    print()
    print(f"{len(names)} part(s) built, validated and written.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
