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
    deck_thickness: float = 2.0
    """Raised from 1.4 after the first prints, and it is the joint that
    raised it, not the road.

    The joint splits the deck at mid-thickness, so every tab is half a deck
    less half a clearance: at 1.4 that is **0.625 mm**, three layers at a
    0.2 mm height, and the printed teeth were thin enough to flex apart, snap
    off while being pushed together, and be hard to part from their supports.
    At 2.0 a tab is 0.925 mm — 3.2 times the bending stiffness, since that
    goes as the cube of thickness.

    It is not free. The envelope is unchanged, so the rails keep less of it:
    `guide_height` falls from 3.3 mm to 2.7, and the section carries 31 % more
    material. Restoring the guide would mean a taller section — raise
    `rail_height_total` to 5.3 — which is a change to what the track *is*,
    not to how it joins, and has not been made here.
    """

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
        """Mid-thickness of the deck. **The joint's split plane** (§6.2).

        One flat plane, and it sits here rather than at the section's
        mid-height. On a U-channel the deck lies wholly below mid-height, so a
        plane there never touches it: the deck halves end up side by side
        sharing nothing but a vertical face, leaving the two thin rail laps as
        the only thing resisting a vertical load. That is a joint no bridge
        survives, and it was built before it was caught.

        Splitting here costs the rails their symmetry — one keeps a thin sliver
        and the other the tall part — which is what `Connector.detent_spacing`
        is about.
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
        if self.deck_thickness >= self.rail_height_total / 2.0:
            raise ValueError(
                "the deck is half the section or more; the joint splits at its "
                "mid-thickness and the rail above that split would be thinner "
                "than the deck lamina it has to hold (§6.2)"
            )


@dataclass(frozen=True)
class Connector:
    """§6.4.

    `fit_clearance` was calibrated against printed parts on the old I-section
    and is carried over: the mating faces it governs — the lap plane, the
    centreline, the tab tips — are mechanically unchanged by the section swap.

    `lap_length` is back to **6.0**. It was halved to 3.0 and the halving was
    a mistake, which two printed loops settled: at 3.0 the teeth broke while
    being pushed together and the joint did not hold.

    The arithmetic that argued for it was not wrong, it was incomplete. A
    shorter tab does stiffen as `1/a³` while the deflection it may take falls
    only as `a²`, so retention scales as `1/a` and goes up — but only if the
    detent shrinks with the lap. Strain at the tab root goes as `3·δ·h / 2a²`,
    so it is `δ/a²` that has to be held, and halving the lap quarters the
    detent a tab can carry. The detent was left at 0.35 and the root strain
    went to 3.6 %, which is past where PLA yields. The tabs did what the
    formula said they would.

    `detent_height` is **0.28**, down from 0.35, because the deck is thicker
    now and strain goes with thickness too. At a 0.925 mm lamina and a 6 mm
    lap that is 1.1 % — a little over the 0.9 % the joint ran at when it
    worked, and about a third of what the 3 mm version was asking. It buys
    roughly three times the retention force of the old thin-deck joint,
    because a tab 1.48 times thicker is 3.2 times stiffer. The Phase 0 comb
    is what settles it properly; these are predictions, not measurements.
    """

    lap_length: float = 6.0
    fit_clearance: float = 0.15
    detent_offset: float = 3.0
    detent_height: float = 0.4
    detent_lead_angle_deg: float = 30.0
    detent_return_angle_deg: float = 60.0

    detent_spacing: float = 0.40
    """Gap between a rail's two detents, as a fraction of the lap.

    There are two per rail because the split (§6.1) leaves the two rails
    **unlike**: the split plane sits in the deck, so one rail keeps the shallow
    part below it and the other keeps the tall part above. A groove has to be
    cut into the material that receives it, and it can only be cut into the
    tall one — sunk into the shallow one it would leave no floor.

    So each port carries its ribs on one rail and its grooves on the other,
    where a split clear of the rails could put one of each on both. That is
    half the engagements, and this is what buys them back: two ribs and two grooves,
    straddling `detent_offset`. Retention is unchanged; what changed is that
    every groove is now in 3.6 mm of material rather than 1.

    Set to 0 for a single detent per rail, which is the older, weaker joint.
    """

    column_count: int = 2
    """How many columns the section is split into, across its whole width.

    **Two**, and the rails are not among them. Earlier versions cut a separate
    column for each rail, so a port came out as six fingers and the two
    outermost were rail plus a sliver of deck — 1.6 mm wide, 0.6 thick, and
    the one carrying the detent ribs. They snapped off while two pieces were
    being pushed together. Widening them helped and missed the point: a rail
    has no business being a finger of its own.

    So a column now runs from the outside of the rail to a seam in the deck,
    and the rail is simply the tall part of the tooth it belongs to. At two
    columns that is one tooth per half of the road, 12 mm wide, each with its
    own rail along the outer edge and a single seam down the centreline —
    which was there anyway, as the gap the two halves pass through.

    Must be even, or the pattern is not odd in x and a port stops mating with
    its own twin. Four is the same joint in narrower teeth; six is where this
    started.
    """

    @property
    def detent_offsets(self) -> tuple[float, ...]:
        """Where along the lap the detents sit, near end first."""
        if self.detent_spacing <= 0.0:
            return (self.detent_offset,)
        half = self.detent_spacing * self.lap_length / 2.0
        return (self.detent_offset - half, self.detent_offset + half)

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
        if self.detent_spacing < 0.0:
            raise ValueError("detent_spacing must not be negative")
        if self.column_count < 2 or self.column_count % 2:
            raise ValueError(
                f"column_count is {self.column_count}; it must be even and at "
                f"least 2, or the column pattern is not odd in x and a port "
                f"stops mating with its own twin (§6.1)"
            )
        if min(self.detent_offsets) <= 0.0:
            raise ValueError(
                "the near detent would sit behind the port plane; "
                "detent_spacing is too wide for detent_offset"
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
