"""Mate two built parts and prove they share no volume. docs/SPEC.md §9.19.

    blender --background --python blender/mate_check.py -- --a curve --b x_junction

This is the strongest statement Phase 3 can make: not that the connector is
symmetric on paper, but that two *finished* solids — booleans, cleanup and all —
actually fit together. It is the Phase 0 coupon check generalised to every part
in the catalogue.

The two pieces are held a probe gap apart first. A joint is designed to touch,
and a contact makes an exact solver emit slivers of float noise that are not
interference; the gap is far smaller than any real clearance, so it clears the
contact without hiding anything a printed part could feel.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from blender.boolean import apply_boolean  # noqa: E402
from blender.build import from_object, reset_scene, to_object  # noqa: E402
from blender.cleanup import clean  # noqa: E402
from blender.run import build_part  # noqa: E402
from parts import port_frames  # noqa: E402
from trackcore import MATE  # noqa: E402
from trackcore.validate import signed_volume  # noqa: E402


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--a", default="straight")
    parser.add_argument("--b", default="straight")
    parser.add_argument("--port-a", type=int, default=1)
    parser.add_argument("--port-b", type=int, default=0)
    parser.add_argument("--probe-gap", type=float, default=1e-2)
    parser.add_argument("--tolerance", type=float, default=1e-4)
    args = parser.parse_args(argv)

    from mathutils import Matrix

    collection = reset_scene()
    a = build_part(args.a, collection)
    b = build_part(args.b, collection)

    frame_a = port_frames(args.a)[args.port_a]
    frame_b = port_frames(args.b)[args.port_b]
    gap = np.eye(4)
    gap[1, 3] = args.probe_gap          # along the port axis, in A's port frame
    placement = frame_a @ gap @ MATE @ np.linalg.inv(frame_b)
    b.matrix_world = Matrix([list(row) for row in placement])

    probe = a.copy()
    probe.data = a.data.copy()
    collection.objects.link(probe)
    apply_boolean(probe, b, "INTERSECT")
    clean(probe)

    shared = from_object(probe)
    volume = 0.0 if len(shared.verts) == 0 else abs(signed_volume(shared))

    print()
    print(f"  {args.a}[{args.port_a}] <-> {args.b}[{args.port_b}]: "
          f"shared {volume:.6g} mm3")
    if volume > args.tolerance:
        lo, hi = shared.bounds()
        print(f"  INTERFERENCE at x[{lo[0]:.3f},{hi[0]:.3f}] "
              f"y[{lo[1]:.3f},{hi[1]:.3f}] z[{lo[2]:.3f},{hi[2]:.3f}]")
        return 1
    print("  mate OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
