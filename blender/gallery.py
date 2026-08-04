"""One clear image per part, auto-framed.

    blender --background --python blender/gallery.py -- --outdir out/parts

Each part is rendered on its own with the camera fitted to its bounding box, so
a 24 mm straight and a 192 mm ramp both fill the frame.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from blender.build import from_object, reset_scene  # noqa: E402
from blender.preview import add_camera, render  # noqa: E402
from blender.run import build_part  # noqa: E402
from parts import CATALOGUE  # noqa: E402

BODY_COLOUR = (0.78, 0.44, 0.22, 1.0)


def shoot(name: str, outdir: str, azimuth: float, elevation: float,
          margin: float, resolution, extra_kwargs=None):
    collection = reset_scene()
    obj = build_part(name, collection, **(extra_kwargs or {}))
    obj.color = BODY_COLOUR

    mesh = from_object(obj)
    lo, hi = mesh.bounds()
    centre = (lo + hi) / 2.0
    scale = float(np.max(hi - lo)) * margin

    add_camera(tuple(centre), 800.0, azimuth, elevation, scale)
    path = os.path.join(outdir, f"{name}.png")
    render(path, resolution)
    print(f"  {name:17s} {hi[0]-lo[0]:6.1f} x {hi[1]-lo[1]:6.1f} x "
          f"{hi[2]-lo[2]:5.1f} mm -> {path}")
    return path


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--outdir", default="out/parts")
    parser.add_argument("--only", default=None, help="comma-separated names")
    parser.add_argument("--azimuth", type=float, default=-58.0)
    parser.add_argument("--elevation", type=float, default=40.0)
    parser.add_argument("--margin", type=float, default=1.18)
    parser.add_argument("--width", type=int, default=1200)
    parser.add_argument("--height", type=int, default=900)
    args = parser.parse_args(argv)

    os.makedirs(args.outdir, exist_ok=True)
    names = args.only.split(",") if args.only else CATALOGUE

    print()
    for name in names:
        shoot(name, args.outdir, args.azimuth, args.elevation, args.margin,
              (args.width, args.height))
    print()
    print(f"{len(names)} image(s) written to {os.path.abspath(args.outdir)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
