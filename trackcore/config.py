"""Dimensions, tolerances and unit handling. docs/SPEC.md §1, §2, §6.4.

Pure Python. Never imports bpy.

Angles are degrees here, at the config boundary, and radians everywhere else
(§1). Convert once, on the way in.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Body:
    """The track cross-section, §2. A **U-channel**: deck at the bottom, rails
    the full height at both edges.

    The outer envelope and the material are unchanged from the earlier
    symmetric I-section (24 x 4.7, 41.52 mm²). What changed is where the
    material sits, and it is worth being clear that this was a printing
    decision before it was a structural one: nothing with rails below the deck
    can lie flat on a bed, which cost a channel-width bridge on every curve and
    twice that on a rounded X. A U lies flat and bridges nothing.

    It happens to be better anyway. Same material, the guide lip above the deck
    doubles, and the section is stiffer because the I wasted its deck at the
    neutral axis.
    """

    width_outer: float = 24.0
    rail_thickness: float = 1.2
    rail_height_total: float = 4.7
    deck_thickness: float = 1.4

    @property
    def half_width(self) -> float:
        return self.width_outer / 2.0

    @property
    def rail_inner(self) -> float:
        return self.half_width - self.rail_thickness

    @property
    def half_height(self) -> float:
        return self.rail_height_total / 2.0

    @property
    def deck_bottom(self) -> float:
        """The bed face. `z = 0` is the section's mid-height, not the deck."""
        return -self.half_height

    @property
    def deck_top(self) -> float:
        """The driving surface."""
        return -self.half_height + self.deck_thickness

    @property
    def deck_mid(self) -> float:
        """Mid-thickness of the deck, where the joint splits it.

        The connector's split is **stepped**, not flat (§6.2): mid-height
        through the rails, mid-deck through the deck. A flat split at `z = 0`
        misses the deck entirely on a U-channel, because the deck is wholly
        below mid-height — which leaves the two thin rail laps as the only
        thing resisting a vertical load, and that is a joint no bridge
        survives.
        """
        return (self.deck_bottom + self.deck_top) / 2.0

    @property
    def deck_lamina(self) -> float:
        """Half the deck: what one tab carries across the road."""
        return self.deck_thickness / 2.0

    @property
    def guide_height(self) -> float:
        """How far the rails stand above the driving surface."""
        return self.half_height - self.deck_top

    @property
    def channel_width(self) -> float:
        return 2.0 * self.rail_inner

    def validate(self) -> None:
        for name in ("width_outer", "rail_thickness", "rail_height_total",
                     "deck_thickness"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.rail_thickness * 2 >= self.width_outer:
            raise ValueError("rails too thick for the track width")
        if self.deck_thickness >= self.rail_height_total:
            raise ValueError("deck must be thinner than the rail height")
        if self.deck_top >= 0.0:
            raise ValueError(
                "the deck must sit wholly below the section mid-height, so that "
                "the rail columns of the joint split clear of it (§6.2)"
            )


@dataclass(frozen=True)
class Connector:
    """§6.4.

    `fit_clearance` was calibrated against printed parts on the old I-section
    and is carried over: the mating faces it governs — the lap plane, the
    centreline, the tab tips — are mechanically unchanged by the section swap.

    `lap_length` is **6.0**, down from 8.0.

    Shortening is not the compromise it looks like. Strain at the tab root goes
    as `3·δ·h / 2a²`, so a shorter lap demands a shallower detent — but the tab
    also stiffens as `1/a³` while the deflection it may take only falls as `a²`,
    so retention scales as `1/a` and goes **up**. At 6.0 with a 0.35 mm detent
    the root strain is the same 0.8 % it was at 8.0 with 0.50 mm, pull-out is
    about a third higher, and the joint is 4 mm shorter — which matters most to
    the shortest parts, where it was most of the piece.

    It could go shorter still: the whole catalogue builds down to 4.0. What
    stops it is print resolution, not mechanics. Below about 0.3 mm a detent is
    one or two layers and will vary part to part, and that is what the Phase 0
    comb measures.
    """

    lap_length: float = 6.0
    fit_clearance: float = 0.15
    detent_offset: float = 3.0
    detent_height: float = 0.35
    detent_lead_angle_deg: float = 30.0
    detent_return_angle_deg: float = 60.0

    def validate(self) -> None:
        for name in ("lap_length", "fit_clearance", "detent_offset",
                     "detent_height"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.detent_lead_angle_deg >= self.detent_return_angle_deg:
            raise ValueError(
                "lead-in must be shallower than the return face, or the joint "
                "is as hard to assemble as it is to pull apart"
            )


@dataclass(frozen=True)
class Tolerances:
    """§4.1, §4.2. These are about the printed part, not about the maths."""

    chord_sag: float = 0.02
    """Maximum deviation between the faceted sweep and the true curve, mm.

    Station density follows from this. There is deliberately no `resolution`
    parameter: resolution is a consequence of tolerance, and tolerance is the
    thing that matters for a printed part.
    """

    min_radius_factor: float = 1.5
    """A sweep folds through itself below half_width * this. §4.1."""

    def min_radius(self, body: Body) -> float:
        return body.half_width * self.min_radius_factor


@dataclass(frozen=True)
class TrackConfig:
    body: Body = field(default_factory=Body)
    connector: Connector = field(default_factory=Connector)
    tolerances: Tolerances = field(default_factory=Tolerances)

    def validate(self) -> None:
        self.body.validate()
        self.connector.validate()

    @property
    def min_radius(self) -> float:
        return self.tolerances.min_radius(self.body)


DEFAULT = TrackConfig()
