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


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    """Build every part once, through the real entry point."""
    outdir = tmp_path_factory.mktemp("out")
    result = subprocess.run(
        [BLENDER, "--background", "--python", "blender/run.py", "--",
         "--all", "--outdir", str(outdir)],
        cwd=REPO, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return outdir


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
def test_a_unioned_hub_matches_its_analytic_volume(built, name):
    """§9.16 against the real boolean rather than against the slab areas.

    The union has to come out at exactly the volume §5.3's decomposition
    predicts. Overlap double-counted, a slab dropped, or a coplanar face
    mishandled would all show up here.
    """
    mesh = read_stl(str(built / f"{name}.stl"))
    expected = HUBS[name]().expected_volume(DEFAULT)
    assert signed_volume(mesh) == pytest.approx(expected, rel=1e-4)


@pytest.mark.parametrize("name", sorted(HUBS))
def test_a_unioned_hub_is_flat_topped_and_the_right_height(built, name):
    mesh = read_stl(str(built / f"{name}.stl"))
    size = mesh.size()
    assert size[2] == pytest.approx(DEFAULT.body.rail_height_total, abs=1e-3)


def test_the_slabs_are_valid_before_the_boolean_too():
    """If an input is already broken, blaming the solver would be wrong."""
    for name in sorted(HUBS):
        piece = build(name, DEFAULT)
        assert piece.needs_union
        for index, solid in enumerate(piece.solids):
            check(solid, name=f"{name} slab {index}")
