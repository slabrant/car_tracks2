"""trackcore — the geometry of the track, with no Blender in it.

docs/SPEC.md §8: **`trackcore` never imports bpy.** Not for purity, for feedback
speed. Every geometry bug the previous attempts shipped was invisible until
somebody opened a viewport; here pytest finds them in under a second, headless.

Phases 1 and 2 are here: Construction A (swept two-port pieces) in `sweep.py`
and Construction B (junctions) in `hub.py`. Connectors are Phase 3.
"""

from .config import DEFAULT, Body, Connector, Tolerances, TrackConfig
from .connector import MATE, applied, port_matrix
from .edge_unit import PROFILE_VERTS, EdgeUnit, profile, profile_area
from .graft import Graft, GraftInvalid, leg_length
from .frames import DegenerateFrame, Frames
from .frames import build as build_frames
from .hub import Arm, Hub, HubInvalid
from .mesh import MeshData, Piece, prism, read_stl, write_stl
from .path import (Arc, Line, Loop, Path, PathDiscontinuous,
                   PathTooTightError, Ramp)
from .sweep import (expected_volume, port_matrices, sweep,
                    swept_with_ports)
from .validate import MeshInvalid, check, signed_volume

__all__ = [
    "DEFAULT", "Body", "Connector", "Tolerances", "TrackConfig",
    "EdgeUnit", "profile", "profile_area", "PROFILE_VERTS",
    "Frames", "build_frames", "DegenerateFrame",
    "MeshData", "Piece", "prism", "read_stl", "write_stl",
    "Hub", "Arm", "HubInvalid",
    "Graft", "GraftInvalid", "leg_length",
    "Path", "Line", "Arc", "Ramp", "Loop", "PathTooTightError",
    "PathDiscontinuous",
    "sweep", "swept_with_ports", "expected_volume", "port_matrices",
    "applied", "port_matrix", "MATE",
    "check", "signed_volume", "MeshInvalid",
]
