"""Mate two built parts and prove the joint is real. docs/SPEC.md §9.19.

    blender --background --python blender/mate_check.py -- --a curve --b x_junction

This is the strongest statement Phase 3 can make: not that the connector is
symmetric on paper, but that two *finished* solids — booleans, cleanup and all —
actually fit together. It is the Phase 0 coupon check generalised to every part
in the catalogue.

Two claims, and the second is the one with teeth:

1. **No interference.** The pieces are held a probe gap apart first. A joint is
   designed to touch, and a contact makes an exact solver emit slivers of float
   noise that are not interference; the gap is far smaller than any real
   clearance, so it clears the contact without hiding anything a printed part
   could feel.
2. **They interleave.** Sharing no volume proves almost nothing on its own —
   two pieces held apart share none, and neither do two flat ends butted
   together. See `interlock`.
"""

from __future__ import annotations

import argparse
import math
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
from trackcore import DEFAULT, MATE, TrackConfig, profile_area  # noqa: E402
from trackcore.mesh import cross_section_area  # noqa: E402
from trackcore.path import DEFAULT_LOOP_DRIFT  # noqa: E402
from trackcore.validate import signed_volume  # noqa: E402


NEARBY = (math.hypot(DEFAULT.body.half_width, DEFAULT.body.half_height)
          + DEFAULT_LOOP_DRIFT - DEFAULT.body.half_width) / 2.0
"""How far from the port axis a section cut still counts, mm. 13.1.

Wide enough to take in all of the section being measured — its furthest corner
is 12.23 mm out — and narrow enough to miss anything else of the same part on
the same plane. The loop sets the upper bound: its two ends pass one drift
apart, so a 30 mm disc read a loop port at 49.8 mm² of a 41.5 mm² section,
counting the run going past as if it were part of the joint.
"""


def interlock(mesh_a, mesh_b, frame, config: TrackConfig = DEFAULT,
              samples: int = 15):
    """Do the two tabs actually interleave, or do the pieces merely touch?

    "Shared volume 0" is necessary and nowhere near sufficient: two pieces held
    apart share nothing, and so do two flat ends butted together. Neither would
    hold a car, let alone a bridge.

    What a lap joint claims is stronger — that over `2 * lap_length` of overlap
    **both** pieces have material on the same plane, each carrying its own tab,
    the two together making up the whole section bar the clearances. That is
    what is measured here, square to the port axis and in the port's own frame
    so it works on a curve as well as a straight.

    Returns a list of (distance, area_a, area_b) with the distance measured
    from the port plane along the outward axis; negative is inside piece A.
    """
    origin = np.asarray(frame)[:3, 3]
    axis = np.asarray(frame)[:3, 1]
    lap = config.connector.lap_length
    detents = config.connector.detent_offsets

    readings = []
    for t in np.linspace(-lap + 0.4, lap - 0.4, samples):
        # skip the detents: there the rib adds to one piece and the groove that
        # receives it takes rather more from the other, by design, so the two
        # do not sum to the section there and it proves nothing either way.
        if any(abs(abs(float(t)) - d) < 0.9 for d in detents):
            continue
        point = origin + axis * float(t)
        readings.append((
            float(t),
            cross_section_area(mesh_a, point, axis, within=NEARBY),
            cross_section_area(mesh_b, point, axis, within=NEARBY),
        ))
    return readings


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

    # before the boolean: it consumes `b`, and the interlock measurement below
    # still needs it
    placed_a = from_object(a)
    placed_b = from_object(b).transformed(placement)

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

    # ... and that they interleave rather than merely failing to collide.
    readings = interlock(placed_a, placed_b, frame_a)
    whole = profile_area(DEFAULT.body)
    worst = min(min(area_a, area_b) for _t, area_a, area_b in readings)
    thinnest = min(area_a + area_b for _t, area_a, area_b in readings)

    print(f"  lap zone: each piece carries at least {worst:.3f} mm², "
          f"together at least {thinnest:.3f} of {whole:.3f}")
    if worst < 0.4 * whole:
        for t, area_a, area_b in readings:
            print(f"    {t:+6.2f} mm   A {area_a:7.3f}   B {area_b:7.3f}")
        print("  NOT INTERLOCKED: one piece runs out inside the lap zone")
        return 1

    print("  mate OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
