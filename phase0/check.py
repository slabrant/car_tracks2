"""Validate exported STLs against docs/SPEC.md §7, without Blender.

    python3 phase0/check.py phase0/out/*.stl

build_coupons.py already validates the solid it builds. This re-checks what
actually landed on disk, because STL stores float32 and the round trip can
collapse vertices that were distinct in the model.
"""

from __future__ import annotations

import glob
import os
import sys

from geom import read_stl
from validate import MeshInvalid, check


def main(argv: list[str]) -> int:
    paths: list[str] = []
    for pattern in argv or ["phase0/out/*.stl"]:
        paths.extend(sorted(glob.glob(pattern)))
    if not paths:
        print("no STL files matched")
        return 1

    failures = 0
    for path in paths:
        mesh = read_stl(path)
        lo, hi = mesh.bounds()
        size = hi - lo
        try:
            stats = check(mesh, name=os.path.basename(path))
            verdict = (f"OK   solids={int(stats['components'])} "
                       f"tris={int(stats['faces']):5d} "
                       f"vol={stats['volume_mm3']:9.2f} mm3")
        except MeshInvalid as exc:
            verdict = f"FAIL {exc}"
            failures += 1
        print(f"{os.path.basename(path):28s} "
              f"{size[0]:6.1f} x {size[1]:5.1f} x {size[2]:5.1f} mm   {verdict}")

    print()
    if failures:
        print(f"{failures} file(s) failed SPEC.md §7")
        return 1
    print(f"all {len(paths)} file(s) passed SPEC.md §7")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    sys.exit(main(sys.argv[1:]))
