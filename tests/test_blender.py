"""docs/SPEC.md §9.24: every part valid, junctions included, after the union.

This is the one test that needs Blender, because the prism union of §5.3 is the
one operation trackcore does not do itself. It is skipped when Blender is
absent, which keeps §9.26 honest: the rest of the suite runs without it.
"""

from __future__ import annotations

import pathlib
import shutil
import subprocess

import pytest

from parts import CATALOGUE, HUBS, build
from trackcore import DEFAULT, check, read_stl
from trackcore.validate import signed_volume

REPO = pathlib.Path(__file__).resolve().parent.parent
BLENDER = shutil.which("blender")

pytestmark = pytest.mark.skipif(BLENDER is None,
                                reason="Blender not on PATH")


def _run(*extra, outdir):
    result = subprocess.run(
        [BLENDER, "--background", "--python", "blender/run.py", "--",
         "--all", "--outdir", str(outdir), *extra],
        cwd=REPO, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return outdir


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    """Every part, connectors and all, through the real entry point."""
    return _run(outdir=tmp_path_factory.mktemp("joined"))


@pytest.fixture(scope="module")
def bare(tmp_path_factory):
    """Every part with flat ends, so the body can be measured on its own."""
    return _run("--no-connectors", outdir=tmp_path_factory.mktemp("bare"))


def test_every_part_builds_validates_and_exports(built):
    for name in CATALOGUE:
        assert (built / f"{name}.stl").exists(), f"{name} was not written"


@pytest.mark.parametrize("name", CATALOGUE)
def test_the_exported_solid_passes_every_rule(built, name):
    """Including after the union, and after the float32 round trip."""
    mesh = read_stl(str(built / f"{name}.stl"))
    stats = check(mesh, name=name)
    assert stats["components"] == 1.0, "a part must be one connected solid"
    assert stats["euler"] == 2.0


@pytest.mark.parametrize("name", sorted(HUBS))
def test_a_unioned_hub_matches_its_analytic_volume(bare, name):
    """§9.16 against the real boolean rather than against the slab areas.

    Measured on the flat-ended body, because a connector deliberately moves
    material around. Overlap double-counted, a slab dropped, or a coplanar face
    mishandled would all show up here.
    """
    mesh = read_stl(str(bare / f"{name}.stl"))
    expected = HUBS[name]().expected_volume(DEFAULT)
    assert signed_volume(mesh) == pytest.approx(expected, rel=1e-4)


@pytest.mark.parametrize("name", CATALOGUE)
def test_a_connector_removes_more_than_it_adds(built, bare, name):
    """A joint is a net cut: two tabs out, two notches and a slot in.

    Not a deep property, but it catches a whole class of wiring mistake — cuts
    silently skipped, or applied in the wrong order so the tabs are eaten.
    """
    joined = signed_volume(read_stl(str(built / f"{name}.stl")))
    plain = signed_volume(read_stl(str(bare / f"{name}.stl")))
    assert 0.5 * plain < joined < plain


@pytest.mark.parametrize("name", sorted(HUBS))
def test_a_unioned_hub_is_flat_topped_and_the_right_height(built, name):
    mesh = read_stl(str(built / f"{name}.stl"))
    size = mesh.size()
    assert size[2] == pytest.approx(DEFAULT.body.rail_height_total, abs=1e-3)


def test_every_boolean_input_is_valid_before_the_solver_sees_it():
    """If an input is already broken, blaming the solver would be wrong."""
    for name in CATALOGUE:
        piece = build(name, DEFAULT)
        assert piece.needs_boolean
        for index, solid in enumerate(piece.every_solid()):
            check(solid, name=f"{name} input {index}")


# -- §9.19 at mesh level -----------------------------------------------------


MATINGS = [
    ("straight", 1, "straight", 0),
    ("curve", 1, "x_junction", 0),
    ("x_rounded", 2, "curve", 0),
    ("y_rounded", 1, "straight", 0),
    ("ramp", 1, "t_rounded", 2),
    ("s_bend", 1, "y_junction", 1),
    ("t_rounded", 0, "x_rounded", 3),
]


@pytest.mark.parametrize("a,port_a,b,port_b", MATINGS)
def test_two_finished_parts_actually_fit_together(a, port_a, b, port_b):
    """The strongest claim Phase 3 can make. Not that the port is symmetric on
    paper — that two real solids, booleans and cleanup included, share no
    volume when mated. Any port mates any port, either way round."""
    result = subprocess.run(
        [BLENDER, "--background", "--python", "blender/mate_check.py", "--",
         "--a", a, "--b", b, "--port-a", str(port_a), "--port-b", str(port_b)],
        cwd=REPO, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "mate OK" in result.stdout
