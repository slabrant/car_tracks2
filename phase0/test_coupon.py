"""Headless tests for the Phase 0 coupon. No Blender required.

    python3 -m pytest phase0/ -q

These are the Phase 0 slice of docs/SPEC.md §9: every boolean input is a valid
solid on its own, and the port mates with a 180-degree rotated copy of itself
with exactly fit_clearance everywhere it should.
"""

from __future__ import annotations

import pathlib
import subprocess

import numpy as np
import pytest

from coupon import Config, comb_values, coupon_parts, solid_aabbs
from geom import box, prism_yz, rotation_z
from validate import MeshInvalid, check

HERE = pathlib.Path(__file__).resolve().parent
TOL = 1e-9


# --------------------------------------------------------------------------
# primitives
# --------------------------------------------------------------------------


def test_box_is_a_valid_solid():
    stats = check(box((-1.0, -2.0, -3.0), (1.0, 2.0, 3.0)), name="box")
    assert stats["volume_mm3"] == pytest.approx(2 * 4 * 6)


def test_prism_is_a_valid_solid():
    triangle = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]  # CCW in (y, z)
    stats = check(prism_yz(triangle, 0.0, 2.0), name="prism")
    assert stats["volume_mm3"] == pytest.approx(0.5 * 2.0)


def test_prism_rejects_wrong_winding():
    clockwise = [(0.0, 0.0), (0.0, 1.0), (1.0, 0.0)]
    with pytest.raises(ValueError):
        prism_yz(clockwise, 0.0, 1.0)


def test_validator_catches_a_hole():
    solid = box((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    holed = type(solid)(verts=solid.verts, faces=solid.faces[:-1])
    with pytest.raises(MeshInvalid):
        check(holed, name="holed")


# --------------------------------------------------------------------------
# every boolean input must itself be a valid solid
# --------------------------------------------------------------------------


@pytest.mark.parametrize("clearance", comb_values())
def test_all_parts_are_valid_solids(clearance):
    cfg = Config(fit_clearance=clearance)
    for label, mesh, _op in coupon_parts(cfg, tally=3):
        check(mesh, name=label)


@pytest.mark.parametrize("clearance", comb_values())
def test_config_accepts_every_comb_value(clearance):
    Config(fit_clearance=clearance).validate()


@pytest.mark.parametrize(
    "kwargs",
    [
        {"detent_offset": 9.0},                                # rib off the tab
        {"detent_height": 3.0},                                # cuts the rail
        {"fit_clearance": -0.1},                               # nonsense
        {"body_length": 1.0},                                  # shorter than notch
        {"rail_thickness": 20.0},                              # wider than track
        {"detent_lead_angle": 70.0, "detent_return_angle": 60.0},  # inverted ramp
        {"detent_offset": 0.2},                                # rib meets groove
    ],
)
def test_config_rejects_bad_geometry(kwargs):
    with pytest.raises(ValueError):
        Config(**kwargs).validate()


# --------------------------------------------------------------------------
# the mating test: SPEC.md §9.19 in Phase 0 form
# --------------------------------------------------------------------------


def _rotate_aabb_180_z(lo, hi):
    """A 180-degree rotation about Z maps (x, y, z) -> (-x, -y, z)."""
    return (-hi[0], -hi[1], lo[2]), (-lo[0], -lo[1], hi[2])


def _overlap(a_lo, a_hi, b_lo, b_hi) -> float:
    """Smallest per-axis overlap. Positive means the boxes interpenetrate."""
    return min(min(a_hi[i], b_hi[i]) - max(a_lo[i], b_lo[i]) for i in range(3))


@pytest.mark.parametrize("clearance", comb_values())
def test_port_mates_with_a_rotated_copy_of_itself(clearance):
    cfg = Config(fit_clearance=clearance)
    piece_a = solid_aabbs(cfg)
    piece_b = [(f"B:{label}", *_rotate_aabb_180_z(lo, hi))
               for label, lo, hi in piece_a]

    for a_label, a_lo, a_hi in piece_a:
        for b_label, b_lo, b_hi in piece_b:
            gap = _overlap(a_lo, a_hi, b_lo, b_hi)
            assert gap <= TOL, (
                f"{a_label} interpenetrates {b_label} by {gap:.4f} mm "
                f"at clearance {clearance}"
            )


@pytest.mark.parametrize("clearance", comb_values())
def test_lap_faces_are_exactly_one_clearance_apart(clearance):
    """The two halves of a rail must clear each other in z, SPEC.md §6.2."""
    cfg = Config(fit_clearance=clearance)
    by_label = {label: (lo, hi) for label, lo, hi in solid_aabbs(cfg)}

    a_lo, _a_hi = by_label["tab_rail_px"]                        # A keeps upper
    _b_lo, b_hi = _rotate_aabb_180_z(*by_label["tab_rail_nx"])   # B keeps lower

    assert a_lo[2] - b_hi[2] == pytest.approx(clearance, abs=1e-12)


@pytest.mark.parametrize("clearance", comb_values())
def test_centreline_slot_is_one_clearance_wide(clearance):
    """The split runs through x = 0, so our (+x) half slides past the mating
    piece's (-x) half and the two need lateral clearance."""
    cfg = Config(fit_clearance=clearance)
    by_label = {label: (lo, hi) for label, lo, hi in solid_aabbs(cfg)}

    a_lo, _a_hi = by_label["tab_deck_px"]                        # A, upper, +x
    _b_lo, b_hi = _rotate_aabb_180_z(*by_label["tab_deck_px"])   # B, upper, -x

    assert a_lo[0] - b_hi[0] == pytest.approx(clearance, abs=1e-12)


@pytest.mark.parametrize("clearance", comb_values())
def test_notch_clears_the_mating_tab_longitudinally(clearance):
    """A tab must never bottom out in its notch, SPEC.md §6.2."""
    cfg = Config(fit_clearance=clearance)
    by_label = {label: (lo, hi) for label, lo, hi in solid_aabbs(cfg)}

    _a_lo, a_hi = by_label["body_rail_px"]
    b_lo, _b_hi = _rotate_aabb_180_z(*by_label["tab_rail_nx"])

    assert b_lo[1] - a_hi[1] == pytest.approx(clearance, abs=1e-12)


# --------------------------------------------------------------------------
# detents: SPEC.md §6.3
# --------------------------------------------------------------------------


def _yz_polygon(mesh) -> list[tuple[float, float]]:
    """Recover the (y, z) cross-section of a prism_yz mesh."""
    n = len(mesh.verts) // 2
    return [(float(v[1]), float(v[2])) for v in mesh.verts[:n]]


def _span_at_z(poly, z: float):
    """The polygon's y-extent at height ``z``, or None if it does not reach."""
    hits: list[float] = []
    n = len(poly)
    for i in range(n):
        y1, z1 = poly[i]
        y2, z2 = poly[(i + 1) % n]
        if abs(z1 - z2) < 1e-15:
            if abs(z1 - z) < 1e-12:
                hits.extend([y1, y2])
            continue
        if (z1 - z) * (z2 - z) <= 0:
            t = (z - z1) / (z2 - z1)
            hits.append(y1 + t * (y2 - y1))
    if not hits:
        return None
    return min(hits), max(hits)


@pytest.mark.parametrize("clearance", comb_values())
def test_rib_seats_inside_the_partner_groove(clearance):
    cfg = Config(fit_clearance=clearance)
    parts = {label: mesh for label, mesh, _ in coupon_parts(cfg)}

    rib = _yz_polygon(parts["rib_px"])
    # the partner's groove_nx, brought into world coordinates by the mating
    # rotation: y -> -y, z unchanged
    groove = [(-y, z) for (y, z) in _yz_polygon(parts["groove_nx"])]

    rib_centre = float(np.mean([p[0] for p in rib]))
    groove_centre = float(np.mean([p[0] for p in groove]))
    assert abs(rib_centre - groove_centre) < 1.0, (
        "rib and partner groove are not at the same station along the joint"
    )

    partner_face = -cfg.lap_face_z
    rib_apex = cfg.lap_face_z - cfg.detent_height
    assert rib_apex < partner_face, "rib does not reach past the partner's lap face"

    for z in np.linspace(partner_face, rib_apex, 12)[1:]:
        rib_span = _span_at_z(rib, float(z))
        groove_span = _span_at_z(groove, float(z))
        assert groove_span is not None, f"groove does not reach z={z:.3f}"
        if rib_span is None:
            continue
        assert groove_span[0] <= rib_span[0] + TOL, (
            f"rib overhangs the groove on the -y side at z={z:.3f}"
        )
        assert groove_span[1] >= rib_span[1] - TOL, (
            f"rib overhangs the groove on the +y side at z={z:.3f}"
        )


@pytest.mark.parametrize("clearance", comb_values())
def test_return_face_is_steeper_than_the_lead_in(clearance):
    """Asymmetric ramp: easy to push together, hard to pull apart, §6.3."""
    cfg = Config(fit_clearance=clearance)
    parts = {label: mesh for label, mesh, _ in coupon_parts(cfg)}
    rib = _yz_polygon(parts["rib_px"])

    apex = min(rib, key=lambda p: p[1])          # rib points down from the face
    base = [p for p in rib if p is not apex]
    lead = max(base, key=lambda p: p[0])         # +y side, insertion
    ret = min(base, key=lambda p: p[0])          # -y side, pull-out

    lead_run = lead[0] - apex[0]
    return_run = apex[0] - ret[0]
    assert lead_run > return_run, (
        f"lead-in run {lead_run:.3f} should exceed return run {return_run:.3f}"
    )


@pytest.mark.parametrize("clearance", comb_values())
def test_no_rib_meets_a_rib(clearance):
    """The longitudinal offset in §6.3 exists precisely to prevent this."""
    cfg = Config(fit_clearance=clearance)
    parts = {label: mesh for label, mesh, _ in coupon_parts(cfg)}

    rib_px = parts["rib_px"].verts
    partner_rib = parts["rib_nx"].transformed(rotation_z(np.pi)).verts

    a_lo, a_hi = rib_px.min(axis=0), rib_px.max(axis=0)
    b_lo, b_hi = partner_rib.min(axis=0), partner_rib.max(axis=0)
    assert _overlap(a_lo, a_hi, b_lo, b_hi) <= TOL


# --------------------------------------------------------------------------
# architecture: SPEC.md §9.25
# --------------------------------------------------------------------------


def test_pure_modules_never_import_bpy():
    needle = "import " + "bpy"  # split so this file does not match itself
    for name in ("geom.py", "coupon.py", "validate.py"):
        source = (HERE / name).read_text()
        assert needle not in source, f"{name} imports bpy"


def test_headless_modules_import_without_blender():
    script = "import geom, coupon, validate; print('ok')"
    result = subprocess.run(
        ["python3", "-c", script], cwd=HERE, capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout
