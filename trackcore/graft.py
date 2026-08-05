"""Construction C: supports. docs/SPEC.md §5.5.

Pure Python + numpy. Never imports bpy.

A support is a short straight with a third port grafted square to it. The leg
that reaches the ground is an **ordinary track piece stood on end** — no new
interface is invented, because §6.1's port is genderless and identical
everywhere, so a straight is already a structural column.

Turned over, a support is its own foot (§5.6): the track section becomes the
base and the leg plugs into the now-upward stub.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import DEFAULT, TrackConfig
from .connector import port_matrix
from .mesh import MeshData, Piece
from .path import Line, Path
from .sweep import port_matrices, sweep


class GraftInvalid(ValueError):
    """A support whose stub or body cannot carry a joint."""


@dataclass(frozen=True)
class Graft:
    """A straight of ``length`` with a stub reaching ``depth`` below the deck."""

    length: float
    depth: float

    # -- paths -----------------------------------------------------------

    def body_path(self) -> Path:
        return Path.chain(Line(self.length))

    def stub_path(self, config: TrackConfig = DEFAULT) -> Path:
        """The stub starts at the driving surface so it never rises into the
        channel, where it would block a car."""
        return Path.chain(Line(config.body.deck_top + self.depth))

    def stub_transform(self, config: TrackConfig = DEFAULT) -> np.ndarray:
        """Place the stub: sweeping downward, section standing across the track.

        The profile's `across` maps to world X and its `up` to world Y, which is
        what lands the stub's rail flanges exactly on the body's own lower rails
        — both occupy `|x| ∈ [rail_inner, half_width]`. The section simply
        continues downward, and the stub comes out an I-beam column by accident
        of the track already being one.
        """
        matrix = np.eye(4)
        matrix[:3, 0] = (1.0, 0.0, 0.0)     # local across -> world +X
        matrix[:3, 1] = (0.0, 0.0, -1.0)    # local tangent -> world -Z
        matrix[:3, 2] = (0.0, 1.0, 0.0)     # local up -> world +Y
        matrix[:3, 3] = (0.0, self.length / 2.0, config.body.deck_top)
        return matrix

    # -- validation ------------------------------------------------------

    def validate(self, config: TrackConfig = DEFAULT) -> None:
        body, connector = config.body, config.connector
        reach = connector.lap_length + connector.fit_clearance
        if self.depth <= reach:
            raise GraftInvalid(
                f"stub depth {self.depth:.1f} mm leaves no room for a joint; "
                f"the notches need {reach:.1f} mm and would cut into the body"
            )
        clear = self.length / 2.0 - body.half_height
        if clear <= reach:
            raise GraftInvalid(
                f"a {self.length:.1f} mm support puts the stub {clear:.1f} mm "
                f"from its own end joints, which need {reach:.1f} mm"
            )

    # -- solids ----------------------------------------------------------

    def solids(self, config: TrackConfig = DEFAULT) -> list[MeshData]:
        config.validate()
        self.validate(config)
        stub = sweep(self.stub_path(config), config)
        return [sweep(self.body_path(), config),
                stub.transformed(self.stub_transform(config))]

    def port_matrices(self, config: TrackConfig = DEFAULT) -> list[np.ndarray]:
        """Three ports: both ends of the body, and the foot of the stub.

        The stub's *upper* port is buried inside the body and gets no connector.
        """
        matrix = self.stub_transform(config)
        stub_far = port_matrices(self.stub_path(config), config)[1]
        return [*port_matrices(self.body_path(), config), matrix @ stub_far]

    def piece(self, name: str, config: TrackConfig = DEFAULT) -> Piece:
        return Piece(name=name, solids=tuple(self.solids(config)))

    # -- geometry a caller may want --------------------------------------

    def stub_port_height(self, config: TrackConfig = DEFAULT) -> float:
        """Where the stub's port plane sits, relative to the deck mid-plane."""
        return -self.depth

    def foot_base_height(self, config: TrackConfig = DEFAULT) -> float:
        """How far the deck mid-plane sits above the ground when used as a foot.

        Turned over, the support rests on its own rails, so this is just half
        the rail height — the same as any piece lying on the floor.
        """
        return config.body.half_height


def leg_length(deck_height: float, depth: float,
               config: TrackConfig = DEFAULT) -> float:
    """The straight needed between a foot and a support to reach ``deck_height``.

    Ground to bridge deck runs: the foot's own half height, its stub depth, the
    leg, then the support's stub depth. Everything but the leg is fixed, so the
    leg is what makes a stack land on a standard height.
    """
    return deck_height - 2.0 * depth
