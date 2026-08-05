"""Build the whole catalogue at several joint configurations.

    blender --background --python blender/sweep_check.py

Every real defect in this project so far was found by hand, not by the suite,
and the reason is always the same: the tests built the catalogue at *one*
configuration. A 45° arc was non-manifold at every lap under 7.4 mm and nothing
noticed, because 8.0 was the only lap anyone built.

This walks the joint's parameter space instead. Exit code is non-zero if any
part fails at any configuration.
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
from parts import CATALOGUE, build  # noqa: E402
from trackcore import Connector, TrackConfig, check  # noqa: E402
from trackcore.validate import MeshInvalid  # noqa: E402

REFERENCE_LAP, REFERENCE_DETENT, REFERENCE_OFFSET = 6.0, 0.35, 3.0


def scaled(lap: float, clearance: float) -> Connector:
    """A joint at this lap, with the detent scaled to hold root strain.

    Comparing laps at a fixed detent height would be comparing a joint against
    one that would never be built: strain at the tab root goes as `δ / a²`, so a
    shorter tab must carry a shallower detent.
    """
    offset = REFERENCE_OFFSET * lap / REFERENCE_LAP
    reference = REFERENCE_LAP + 0.15 + REFERENCE_OFFSET
    a = lap + clearance + offset
    deflection = (REFERENCE_DETENT - 0.15) * (a / reference) ** 2
    return Connector(lap_length=lap, fit_clearance=clearance,
                     detent_offset=offset,
                     detent_height=round(deflection + clearance, 3))


def buildable(name: str, config: TrackConfig) -> str | None:
    collection = reset_scene()
    try:
        piece = build(name, config)
        target = to_object(piece.solids[0], name, collection)
        for operation, meshes in piece.stages():
            for index, mesh in enumerate(meshes):
                tool = to_object(mesh, f"{name}_{operation}_{index}", collection)
                apply_boolean(target, tool, operation)
        clean(target)
        check(from_object(target), name=name)
        return None
    except (MeshInvalid, ValueError) as exc:
        return str(exc).split(";")[0]


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--laps", type=float, nargs="*",
                        default=[4.0, 5.0, 6.0, 8.0])
    parser.add_argument("--clearances", type=float, nargs="*",
                        default=[0.10, 0.20])
    args = parser.parse_args(argv)

    failures = 0
    print()
    for lap in args.laps:
        for clearance in args.clearances:
            connector = scaled(lap, clearance)
            config = TrackConfig(connector=connector)
            bad = {name: reason for name in CATALOGUE
                   if (reason := buildable(name, config)) is not None}
            label = (f"lap {lap:4.1f}  clearance {clearance:4.2f}  "
                     f"detent {connector.detent_height:5.3f}")
            if bad:
                failures += len(bad)
                print(f"  {label}  FAIL")
                for name, reason in bad.items():
                    print(f"      {name}: {reason}")
            else:
                print(f"  {label}  all {len(CATALOGUE)} OK")

    print()
    if failures:
        print(f"{failures} part/configuration combination(s) failed")
        return 1
    total = len(args.laps) * len(args.clearances) * len(CATALOGUE)
    print(f"{total} part/configuration combinations, all valid")
    return 0


if __name__ == "__main__":
    sys.exit(main())
