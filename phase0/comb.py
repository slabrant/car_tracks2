"""Phase 0 recalibration comb for the U-channel section. docs/SPEC.md §10.

    blender --background --python phase0/comb.py -- --outdir out/comb

Two axes on one plate:

- `fit_clearance`, which must be re-measured because the value in `Connector`
  was calibrated against the old I-section joint.
- `lap_length`, which has never been measured at all. Shorter is predicted to
  hold *better*, not worse — stiffness grows as 1/a³ while the allowed
  deflection only falls as a², so retention scales as 1/a — with print
  resolution of the detent as the real floor.

`detent_height` is not a free axis. It is scaled with the lap so that root
strain stays at the 0.8 % the current joint runs at, which is the only way the
two laps are comparable: otherwise a short lap is being tested with a detent it
would never be built with.

Coupons are ordinary straights built through the real `trackcore` path, so what
is printed is what the catalogue ships. Two per combination, because the port is
genderless and a coupon mates with its twin.
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
from blender.export import export  # noqa: E402
from trackcore import (DEFAULT, Connector, Line, Path, Piece, TrackConfig,
                       applied, check, port_matrices, sweep)
from trackcore.mesh import box, translation  # noqa: E402

CLEARANCES = [0.10, 0.15, 0.20]
LAPS = [5.0, 6.0, 8.0]

REFERENCE_LAP, REFERENCE_OFFSET, REFERENCE_DEFLECTION = 8.0, 4.0, 0.35
"""The joint as built: 8 mm lap, detent 0.50, clearance 0.15 -> 0.35 mm of
interference, about 0.8 % strain at the tab root."""

COUPON_LENGTH = 32.0
TALLY_DEPTH, TALLY_WIDTH, TALLY_HEIGHT, TALLY_PITCH = 0.6, 1.0, 2.0, 2.0


def detent_for(lap: float, clearance: float) -> tuple[float, float]:
    """Detent height and offset for a lap, holding root strain constant.

    The tab is a cantilever of length `(lap + clearance) + offset`, and strain
    goes as `deflection / a²`, so the deflection a shorter tab may take falls
    with `a²`. Interference is `detent_height - clearance`, hence the sum.
    """
    offset = REFERENCE_OFFSET * lap / REFERENCE_LAP
    reference_a = REFERENCE_LAP + DEFAULT.connector.fit_clearance + REFERENCE_OFFSET
    a = lap + clearance + offset
    deflection = REFERENCE_DEFLECTION * (a / reference_a) ** 2
    return deflection + clearance, offset


def tally(config: TrackConfig, clearance_index: int, lap_index: int):
    """Pockets in the +X rail's outer face: clearance marks, then lap marks."""
    body = config.body
    reach = config.connector.lap_length + config.connector.fit_clearance
    marks = []
    for group, count in ((0, clearance_index), (1, lap_index)):
        base = reach + 2.0 + group * (COUPON_LENGTH - 2.0 * reach - 4.0) / 2.0
        for k in range(count):
            y = base + k * TALLY_PITCH
            marks.append(box(
                (body.half_width - TALLY_DEPTH, y, -TALLY_HEIGHT / 2.0),
                (body.half_width + 0.01, y + TALLY_WIDTH, TALLY_HEIGHT / 2.0)))
    return marks


def coupon(clearance: float, lap: float, clearance_index: int,
           lap_index: int) -> tuple[Piece, TrackConfig]:
    height, offset = detent_for(lap, clearance)
    config = TrackConfig(connector=Connector(
        lap_length=lap, fit_clearance=clearance,
        detent_offset=offset, detent_height=round(height, 3)))

    path = Path.chain(Line(COUPON_LENGTH))
    cuts, adds = applied(port_matrices(path, config), config)
    name = f"c{clearance:.2f}_l{lap:.0f}"
    piece = Piece(name=name, solids=(sweep(path, config),),
                  cuts=tuple(cuts) + tuple(tally(config, clearance_index,
                                                 lap_index)),
                  additions=tuple(adds))
    return piece, config


def build(piece: Piece, collection, offset_x: float, offset_y: float = 0.0):
    from mathutils import Matrix
    target = to_object(piece.solids[0], piece.name, collection)
    for operation, meshes in piece.stages():
        for index, mesh in enumerate(meshes):
            tool = to_object(mesh, f"{piece.name}_{operation}_{index}",
                             collection)
            apply_boolean(target, tool, operation)
    clean(target)
    stats = check(from_object(target), name=piece.name)
    target.matrix_world = Matrix(
        [list(row) for row in translation(offset_x, offset_y, 0.0)])
    return target, stats


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--outdir", default="out/comb")
    parser.add_argument("--pairs", type=int, default=2)
    parser.add_argument("--gap", type=float, default=6.0)
    parser.add_argument("--bed", type=float, default=220.0,
                        help="bed width, mm; the plate wraps to fit")
    args = parser.parse_args(argv)

    os.makedirs(args.outdir, exist_ok=True)
    collection = reset_scene()
    pitch_x = DEFAULT.body.width_outer + args.gap
    pitch_y = COUPON_LENGTH + args.gap
    columns = max(1, int(args.bed // pitch_x))
    plate, index = [], 0

    print()
    print(f"{'combo':14s} {'clear':>6s} {'lap':>5s} {'detent':>7s} "
          f"{'offset':>7s} {'marks':>7s}  status")
    for ci, clearance in enumerate(CLEARANCES, start=1):
        for li, lap in enumerate(LAPS, start=1):
            piece, config = coupon(clearance, lap, ci, li)
            for _ in range(args.pairs):
                column, row = index % columns, index // columns
                obj, stats = build(piece, collection,
                                   column * pitch_x, row * pitch_y)
                plate.append(obj)
                index += 1
            detent = config.connector
            print(f"{piece.name:14s} {clearance:6.2f} {lap:5.1f} "
                  f"{detent.detent_height:7.3f} {detent.detent_offset:7.2f} "
                  f"{ci}+{li:<5d}  OK  vol={stats['volume_mm3']:7.1f} mm3")

    rows = (len(plate) + columns - 1) // columns
    span_x, span_y = (min(len(plate), columns) - 1) * pitch_x, (rows - 1) * pitch_y
    centre = translation(-span_x / 2.0, -span_y / 2.0, 0.0)
    from mathutils import Matrix
    for obj in plate:
        obj.matrix_world = Matrix([list(row) for row in centre]) @ obj.matrix_world
    export(plate, os.path.join(args.outdir, "comb_all.stl"))

    print()
    print(f"  {len(plate)} coupons in {rows} rows of up to {columns}, "
          f"{span_x + DEFAULT.body.width_outer:.0f} x "
          f"{span_y + COUPON_LENGTH:.0f} mm, "
          f"{len(CLEARANCES) * len(LAPS)} combinations")
    print(f"  marks on the +X rail: first group = clearance "
          f"({', '.join(f'{c:.2f}' for c in CLEARANCES)}), "
          f"second = lap ({', '.join(f'{l:.0f}' for l in LAPS)})")
    print(f"  written to {os.path.abspath(args.outdir)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
