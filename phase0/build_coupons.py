"""Build the Phase 0 coupon comb in Blender and export STLs.

Run headless:

    blender --background --python phase0/build_coupons.py -- --orientation side

This is the only file in Phase 0 that imports bpy. Everything it needs to
decide is computed in coupon.py and checked in validate.py, both of which run
on plain python.
"""

from __future__ import annotations

import argparse
import os
import sys

import bpy  # noqa: F401  (only available inside Blender)
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from coupon import (  # noqa: E402
    DIFFERENCE,
    Config,
    comb_values,
    coupon_parts,
    orientation_matrix,
    plate_offsets,
)
from geom import MeshData  # noqa: E402
from validate import MeshInvalid, check  # noqa: E402

COLLECTION = "Phase0"


# --------------------------------------------------------------------------
# scene plumbing
# --------------------------------------------------------------------------


def reset_scene() -> "bpy.types.Collection":
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)
    if COLLECTION in bpy.data.collections:
        col = bpy.data.collections[COLLECTION]
    else:
        col = bpy.data.collections.new(COLLECTION)
        bpy.context.scene.collection.children.link(col)
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.length_unit = "MILLIMETERS"
    return col


def to_object(mesh_data: MeshData, name: str, collection) -> "bpy.types.Object":
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    mesh.from_pydata([tuple(v) for v in mesh_data.verts], [], mesh_data.faces)
    mesh.validate(verbose=False)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    collection.objects.link(obj)
    return obj


def from_object(obj) -> MeshData:
    mesh = obj.data
    verts = np.array([tuple(v.co) for v in mesh.vertices], dtype=np.float64)
    faces = [list(p.vertices) for p in mesh.polygons]
    return MeshData(verts=verts, faces=faces)


def apply_boolean(target, tool, operation: str) -> None:
    """Apply one boolean with the MANIFOLD solver and bake the result."""
    mod = target.modifiers.new(name=f"bool_{operation.lower()}", type="BOOLEAN")
    mod.operation = operation
    mod.object = tool
    try:
        mod.solver = "MANIFOLD"
    except TypeError:
        mod.solver = "EXACT"

    depsgraph = bpy.context.evaluated_depsgraph_get()
    baked = bpy.data.meshes.new_from_object(target.evaluated_get(depsgraph))
    old = target.data
    target.data = baked
    target.modifiers.clear()
    bpy.data.meshes.remove(old)
    bpy.data.objects.remove(tool, do_unlink=True)


# --------------------------------------------------------------------------
# the coupon
# --------------------------------------------------------------------------


def build_coupon(cfg: Config, tally: int, collection, name: str):
    parts = coupon_parts(cfg, tally=tally)
    label0, mesh0, op0 = parts[0]
    if op0 == DIFFERENCE:
        raise RuntimeError("first part must be a UNION; it seeds the solid")

    target = to_object(mesh0, f"{name}_{label0}", collection)
    for label, mesh, op in parts[1:]:
        tool = to_object(mesh, f"{name}_{label}_tool", collection)
        apply_boolean(target, tool, op)
    target.name = name
    return target


def mate_check(cfg: Config, collection, name: str = "mate",
               probe_gap: float = 1e-3,
               dump: str | None = None) -> tuple[float, str]:
    """Intersect a coupon with a 180-degree rotated copy of itself.

    SPEC.md §9.19 at mesh level rather than bounding-box level: this exercises
    the real tabs, notches, ribs and grooves.

    The two pieces are held ``probe_gap`` apart along the joint axis first. The
    deck faces are *designed* to touch — they are the datum that sets piece
    spacing, §6.2 — and a coplanar contact makes the solver emit a sliver of
    float noise that is not an interference. One micron is 100 times smaller
    than the tightest clearance in the comb, so it clears the contact plane
    without hiding any interference that could matter to a printed part.
    """
    from mathutils import Matrix

    piece_a = build_coupon(cfg, tally=0, collection=collection, name=f"{name}_a")
    piece_b = build_coupon(cfg, tally=0, collection=collection, name=f"{name}_b")
    piece_b.matrix_world = (Matrix.Translation((0.0, probe_gap, 0.0))
                            @ Matrix.Rotation(np.pi, 4, "Z"))

    probe = piece_a.copy()
    probe.data = piece_a.data.copy()
    collection.objects.link(probe)

    apply_boolean(probe, piece_b, "INTERSECT")
    shared = from_object(probe)
    if len(shared.verts) == 0:
        volume, where = 0.0, ""
    else:
        volume = abs(_volume(shared))
        lo, hi = shared.bounds()
        thinnest = min(hi - lo)
        where = (f" [{len(shared.verts)}v {len(shared.faces)}f, "
                 f"thinnest axis {thinnest:.2e} mm] at "
                 f"x[{lo[0]:.3f},{hi[0]:.3f}] "
                 f"y[{lo[1]:.3f},{hi[1]:.3f}] z[{lo[2]:.3f},{hi[2]:.3f}]")

    if dump and len(shared.verts):
        from geom import write_stl

        write_stl(shared, dump, name="mate_intersection")
        print(f"    wrote intersection to {dump}")

    bpy.data.objects.remove(probe, do_unlink=True)
    bpy.data.objects.remove(piece_a, do_unlink=True)
    return volume, where


def flip_deviation(mesh: MeshData, decimals: int = 4) -> float:
    """SPEC.md §9.18: rotating a piece 180 degrees about its long axis must
    give back the same part.

    That rotation maps (x, y, z) -> (-x, y, -z). If the connector were gendered,
    or asymmetric in z, the vertex sets would not agree.
    """
    original = np.round(mesh.verts, decimals)
    flipped = np.round(mesh.verts * np.array([-1.0, 1.0, -1.0]), decimals)
    if len(original) != len(flipped):
        return float("inf")

    order_a = np.lexsort((original[:, 2], original[:, 1], original[:, 0]))
    order_b = np.lexsort((flipped[:, 2], flipped[:, 1], flipped[:, 0]))
    return float(np.abs(original[order_a] - flipped[order_b]).max())


def _volume(mesh: MeshData) -> float:
    from validate import signed_volume

    return signed_volume(mesh)


def export(objects, path: str) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    try:
        bpy.ops.wm.stl_export(
            filepath=path, export_selected_objects=True, global_scale=1.0
        )
    except AttributeError:
        bpy.ops.export_mesh.stl(filepath=path, use_selection=True, global_scale=1.0)


# --------------------------------------------------------------------------
# driver
# --------------------------------------------------------------------------


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Phase 0 coupon comb")
    parser.add_argument("--orientation", default="side",
                        choices=["side", "flat", "model"])
    parser.add_argument("--outdir", default="phase0/out")
    parser.add_argument("--gap", type=float, default=8.0,
                        help="mm between coupons on the plate")
    parser.add_argument("--clearances", type=float, nargs="*", default=None,
                        help="override the comb values, in mm")
    parser.add_argument("--pairs", type=int, default=2,
                        help="coupons per clearance (2 lets you mate them)")
    parser.add_argument("--probe-gap", type=float, default=1e-2,
                        help="mm the mate check holds the pieces apart")
    parser.add_argument("--dump-mate", default=None,
                        help="write the mate intersection to this STL")
    parser.add_argument("--mate-tol", type=float, default=1e-6,
                        help="mm3 of shared volume tolerated as solver noise")
    return parser.parse_args(argv)


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parse_args(argv)

    clearances = args.clearances if args.clearances else comb_values()
    os.makedirs(args.outdir, exist_ok=True)

    collection = reset_scene()
    failures: list[str] = []
    plate: list = []
    plate_cursor = 0.0

    print()
    print(f"orientation : {args.orientation}")
    print(f"output      : {os.path.abspath(args.outdir)}")
    print()

    for index, clearance in enumerate(clearances, start=1):
        cfg = Config(fit_clearance=clearance)
        tag = f"c{clearance:.2f}"
        name = f"coupon_{tag}"

        obj = build_coupon(cfg, tally=index, collection=collection, name=name)

        # §7 validation on exactly the solid that is about to be written.
        mesh_data = from_object(obj)
        try:
            stats = check(mesh_data, name=name)
            status = (f"OK   V={int(stats['verts']):5d} F={int(stats['faces']):5d} "
                      f"vol={stats['volume_mm3']:8.2f} mm3")
        except MeshInvalid as exc:
            status = f"FAIL {exc}"
            failures.append(str(exc))

        flip = flip_deviation(mesh_data)
        if flip > 1e-4:
            failures.append(f"{name}: FLIP FAIL, deviation {flip:.6g} mm")
            flip_note = f"FLIP FAIL {flip:.4g} mm"
        else:
            flip_note = "flip OK"

        shared, where = mate_check(cfg, collection, name=f"mate_{tag}",
                                   probe_gap=args.probe_gap,
                                   dump=args.dump_mate)
        if shared > args.mate_tol:
            mate = f"MATE FAIL: pieces share {shared:.6g} mm3{where}"
            failures.append(f"{name}: {mate}")
        else:
            mate = "mate OK"

        print(f"  clearance {clearance:.2f}  tally {index}  {status}  "
              f"{flip_note}  {mate}")

        # lay the pair out and write this clearance's own file
        orient = orientation_matrix(cfg, args.orientation)
        copies = []
        for offset in plate_offsets(cfg, args.orientation, args.pairs, args.gap):
            copy = obj.copy()
            copy.data = obj.data.copy()
            collection.objects.link(copy)
            copy.matrix_world = _to_bpy_matrix(offset @ orient)
            copies.append(copy)

        export(copies, os.path.join(args.outdir, f"{name}_pair.stl"))

        # and stash a set for the combined comb plate
        pitch = (cfg.rail_height_total if args.orientation == "side"
                 else cfg.width_outer) + args.gap
        for k, copy in enumerate(copies):
            comb_copy = copy.copy()
            comb_copy.data = copy.data.copy()
            collection.objects.link(comb_copy)
            comb_copy.matrix_world = _to_bpy_matrix(
                _translate(plate_cursor + k * pitch, 0.0, 0.0) @ orient
            )
            plate.append(comb_copy)
        plate_cursor += pitch * args.pairs + args.gap

        bpy.data.objects.remove(obj, do_unlink=True)
        for copy in copies:
            bpy.data.objects.remove(copy, do_unlink=True)

    if plate:
        centre = _translate(-plate_cursor / 2.0, 0.0, 0.0)
        for obj in plate:
            obj.matrix_world = _to_bpy_matrix(centre) @ obj.matrix_world
        export(plate, os.path.join(args.outdir, "comb_all.stl"))
        print()
        print(f"  comb_all.stl  {len(plate)} coupons, "
              f"{plate_cursor:.1f} mm across")

    print()
    if failures:
        print(f"FAILED: {len(failures)} coupon(s) did not pass SPEC.md §7")
        for line in failures:
            print(f"  - {line}")
        return 1
    print("All coupons passed SPEC.md §7 validation.")
    return 0


def _translate(dx: float, dy: float, dz: float) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (dx, dy, dz)
    return m


def _to_bpy_matrix(m: np.ndarray):
    from mathutils import Matrix

    return Matrix([list(row) for row in m])


if __name__ == "__main__":
    sys.exit(main())
