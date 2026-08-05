"""docs/SPEC.md §9.9–9.11 and §7: the swept solid."""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import PATHS, curve, ramp, s_bend, straight
from trackcore import (DEFAULT, Arc, Line, MeshData, Path, PathTooTightError,
                       TrackConfig, check, expected_volume, profile_area, sweep)
from trackcore.config import Body, Tolerances
from trackcore.edge_unit import PROFILE_VERTS
from trackcore.validate import MeshInvalid, signed_volume

BODY = DEFAULT.body


# -- 9.9 ---------------------------------------------------------------------


def test_a_straight_has_the_expected_bounding_box():
    mesh = sweep(straight(84.0))
    assert mesh.size() == pytest.approx([BODY.width_outer, 84.0,
                                         BODY.rail_height_total], abs=1e-9)


def test_a_straight_uses_two_stations_and_no_more():
    mesh = sweep(straight(84.0))
    assert len(mesh.verts) == 2 * PROFILE_VERTS


# -- 9.10 --------------------------------------------------------------------


def test_a_straight_has_exactly_the_analytic_volume():
    mesh = sweep(straight(84.0))
    ideal = profile_area(BODY) * 84.0
    assert signed_volume(mesh) == pytest.approx(ideal, rel=1e-6)


# -- 9.11 --------------------------------------------------------------------


def test_an_arc_volume_matches_pappus_within_the_faceting_error():
    """A profile centred on the path sweeps area x length exactly. The faceted
    mesh chords the curve, so it comes in slightly under."""
    path = curve(radius=100.0, angle_deg=90.0)
    volume = signed_volume(sweep(path))
    ideal = expected_volume(path)
    assert volume == pytest.approx(ideal, rel=1e-3)
    assert volume < ideal, "chords cannot enclose more than the true curve"


def test_tightening_the_sag_tolerance_converges_on_the_ideal():
    path = curve(radius=100.0, angle_deg=90.0)
    ideal = expected_volume(path)
    coarse = TrackConfig(tolerances=Tolerances(chord_sag=0.5))
    fine = TrackConfig(tolerances=Tolerances(chord_sag=0.002))
    coarse_error = abs(signed_volume(sweep(path, coarse)) - ideal)
    fine_error = abs(signed_volume(sweep(path, fine)) - ideal)
    assert fine_error < coarse_error


# -- §7 on every part --------------------------------------------------------


@pytest.mark.parametrize("name", sorted(PATHS))
def test_every_swept_part_is_a_valid_solid(name):
    stats = check(sweep(PATHS[name]()), name=name)
    assert stats["components"] == 1.0
    assert stats["euler"] == 2.0
    assert stats["volume_mm3"] > 0.0


@pytest.mark.parametrize("angle_deg", [15.0, 45.0, 90.0, 180.0, -90.0])
def test_arcs_of_many_angles_are_valid_solids(angle_deg):
    check(sweep(curve(radius=60.0, angle_deg=angle_deg)), name="arc")


def test_a_banked_curve_is_a_valid_solid():
    check(sweep(curve(radius=100.0, angle_deg=90.0, bank_deg=15.0)),
          name="banked")


def test_a_ramp_is_a_valid_solid():
    check(sweep(ramp(run=84.0, rise=34.0)), name="ramp")


def test_an_s_bend_is_a_valid_solid():
    check(sweep(s_bend()), name="s_bend")


# -- topology ----------------------------------------------------------------


def test_the_mesh_is_manifold_by_construction_not_by_luck():
    """N stations x the profile, quad strips, two caps. §4.3."""
    path = curve(radius=100.0, angle_deg=90.0)
    mesh = sweep(path)
    n_stations = len(mesh.verts) // PROFILE_VERTS

    assert len(mesh.verts) == n_stations * PROFILE_VERTS
    assert len(mesh.faces) == (n_stations - 1) * PROFILE_VERTS + 2
    assert sum(1 for f in mesh.faces if len(f) == 4) == (n_stations - 1) * PROFILE_VERTS
    assert sum(1 for f in mesh.faces if len(f) == PROFILE_VERTS) == 2


def test_end_caps_face_along_the_path_at_each_end():
    path = straight(84.0)
    mesh = sweep(path)
    caps = [f for f in mesh.faces if len(f) == 12]

    for cap in caps:
        pts = mesh.verts[cap]
        normal = np.zeros(3)
        for i in range(len(pts)):
            a, b = pts[i], pts[(i + 1) % len(pts)]
            normal += np.cross(a, b)
        normal /= np.linalg.norm(normal)
        centre = pts.mean(axis=0)
        expected = 1.0 if centre[1] > 42.0 else -1.0
        assert normal[1] == pytest.approx(expected, abs=1e-9)


def test_the_sweep_starts_at_the_path_origin_facing_plus_y():
    mesh = sweep(straight(50.0))
    lo, _hi = mesh.bounds()
    assert lo[1] == pytest.approx(0.0, abs=1e-12)


# -- guards ------------------------------------------------------------------


def test_sweep_re_checks_curvature_against_the_real_profile():
    """A path legal for a narrow track is not legal for a wide one."""
    path = Path.chain(Arc(radius=12.0, angle=math.pi / 2, min_radius=5.0))
    sweep(path, TrackConfig(body=Body(width_outer=12.0)))
    with pytest.raises(PathTooTightError):
        sweep(path, DEFAULT)


def test_sweep_rejects_an_invalid_body():
    with pytest.raises(ValueError):
        sweep(straight(50.0), TrackConfig(body=Body(rail_thickness=20.0)))


def test_validator_catches_a_hole_in_a_swept_mesh():
    mesh = sweep(straight(50.0))
    holed = MeshData(verts=mesh.verts, faces=mesh.faces[:-1])
    with pytest.raises(MeshInvalid):
        check(holed, name="holed")


# -- round trip --------------------------------------------------------------


def test_stl_round_trip_survives_float32(tmp_path):
    from trackcore import read_stl, write_stl

    mesh = sweep(curve(radius=100.0, angle_deg=90.0))
    out = tmp_path / "curve.stl"
    write_stl(mesh, str(out))
    reloaded = read_stl(str(out))

    check(reloaded, name="reloaded")
    assert signed_volume(reloaded) == pytest.approx(signed_volume(mesh), rel=1e-4)


def test_a_long_chain_of_pieces_stays_valid():
    path = Path.chain(Line(84.0), Arc(100.0, math.radians(45.0)),
                      Line(42.0), Arc(100.0, math.radians(-45.0)),
                      Line(84.0))
    check(sweep(path), name="long chain")
