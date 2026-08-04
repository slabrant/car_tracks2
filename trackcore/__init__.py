"""trackcore — the geometry of the track, with no Blender in it.

docs/SPEC.md §8: **`trackcore` never imports bpy.** Not for purity, for feedback
speed. Every geometry bug the previous attempts shipped was invisible until
somebody opened a viewport; here pytest finds them in under a second, headless.

Phase 1 covers Construction A, the swept two-port pieces. Construction B
(junctions) and connectors are later phases.
"""

from .config import DEFAULT, Body, Connector, Tolerances, TrackConfig
from .edge_unit import PROFILE_VERTS, EdgeUnit, profile, profile_area
from .frames import DegenerateFrame, Frames
from .frames import build as build_frames
from .mesh import MeshData, read_stl, write_stl
from .path import Arc, Line, Path, PathDiscontinuous, PathTooTightError, Ramp
from .sweep import expected_volume, sweep
from .validate import MeshInvalid, check, signed_volume

__all__ = [
    "DEFAULT", "Body", "Connector", "Tolerances", "TrackConfig",
    "EdgeUnit", "profile", "profile_area", "PROFILE_VERTS",
    "Frames", "build_frames", "DegenerateFrame",
    "MeshData", "read_stl", "write_stl",
    "Path", "Line", "Arc", "Ramp", "PathTooTightError", "PathDiscontinuous",
    "sweep", "expected_volume",
    "check", "signed_volume", "MeshInvalid",
]
