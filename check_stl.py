"""Validate exported STLs against docs/SPEC.md §7. No Blender needed.

    python3 check_stl.py out/set/*.stl

`blender/run.py` already validates every part before writing it. This re-checks
what actually landed on disk, because STL stores float32 and the round trip can
collapse vertices that were distinct in the model.
"""

from __future__ import annotations

import glob
import os
import sys

from trackcore import check, read_stl
from trackcore.validate import MeshInvalid


def main(argv: list[str]) -> int:
    # A part is one solid and §7 rule 7 says so. A build plate is deliberately
    # many, so it has to say which it is being handed.
    plate = "--plate" in argv
    argv = [a for a in argv if a != "--plate"]

    paths: list[str] = []
    for pattern in argv or ["out/set/*.stl"]:
        paths.extend(sorted(glob.glob(pattern)))
    if not paths:
        print("no STL files matched")
        return 1

    failures = 0
    for path in paths:
        mesh = read_stl(path)
        size = mesh.size()
        try:
            stats = check(mesh, name=os.path.basename(path),
                          components=None if plate else 1)
            verdict = (f"OK   solids={int(stats['components'])} "
                       f"tris={int(stats['faces']):5d} "
                       f"vol={stats['volume_mm3']:9.2f} mm3")
        except MeshInvalid as exc:
            verdict = f"FAIL {exc}"
            failures += 1
        print(f"{os.path.basename(path):24s} "
              f"{size[0]:6.1f} x {size[1]:6.1f} x {size[2]:5.1f} mm   {verdict}")

    print()
    if failures:
        print(f"{failures} file(s) failed SPEC.md §7")
        return 1
    print(f"all {len(paths)} file(s) passed SPEC.md §7")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    sys.exit(main(sys.argv[1:]))
