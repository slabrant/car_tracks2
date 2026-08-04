"""docs/SPEC.md §9.25–9.26: the dependency rule.

`trackcore` never imports bpy. Not for purity — for feedback speed. Every
geometry bug the previous attempts shipped was invisible until somebody opened
a Blender viewport.
"""

from __future__ import annotations

import os
import pathlib
import subprocess
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
NEEDLE = "import " + "bpy"  # split so this file does not match itself


# -- 9.25 --------------------------------------------------------------------


@pytest.mark.parametrize("package", ["trackcore", "parts"])
def test_the_core_never_imports_bpy(package):
    for source in sorted((REPO / package).rglob("*.py")):
        assert NEEDLE not in source.read_text(), f"{source} imports bpy"


def test_bpy_lives_only_in_the_blender_package():
    offenders = [
        p.relative_to(REPO)
        for p in REPO.rglob("*.py")
        if NEEDLE in p.read_text()
        and p.relative_to(REPO).parts[0] not in ("blender", "phase0", ".venv")
    ]
    assert not offenders, f"bpy leaked outside blender/: {offenders}"


# -- 9.26 --------------------------------------------------------------------


def test_the_core_imports_with_blender_absent_from_path():
    env = dict(os.environ)
    env["PATH"] = "/nonexistent"
    env["PYTHONPATH"] = str(REPO)
    script = (
        "import trackcore, parts;"
        "m = trackcore.sweep(parts.straight(84.0));"
        "trackcore.check(m);"
        "print('ok', len(m.verts))"
    )
    result = subprocess.run([sys.executable, "-c", script],
                            capture_output=True, text=True, env=env, cwd=REPO)
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout


def test_the_core_depends_on_nothing_but_numpy():
    allowed = {"numpy", "dataclasses", "typing", "math", "struct",
               "collections", "__future__", "trackcore"}
    for source in sorted((REPO / "trackcore").glob("*.py")):
        for line in source.read_text().splitlines():
            line = line.strip()
            if not line.startswith(("import ", "from ")):
                continue
            if line.startswith("from ."):
                continue
            root = line.split()[1].split(".")[0]
            assert root in allowed, f"{source.name} imports {root}"
