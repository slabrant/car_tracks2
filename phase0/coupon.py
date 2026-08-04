"""Phase 0 calibration coupon, docs/SPEC.md §10.

A stub of track with one port on it. Two coupons mate with each other, because
the port is genderless (§6.1). Printing the comb at a range of ``fit_clearance``
values answers the one question Phase 0 exists to answer.

Pure Python + numpy. Never imports bpy.

The port is the full diagonal split of §6.1: the entire cross-section is cut at
x = 0 and z = 0, and each piece keeps the (+x, +z) and (-x, -z) quadrants. That
is the minimal solution to "genderless and flippable", applied to the whole
section rather than only the rails.

The coupon is deliberately built as a union of boxes and prisms rather than by
sweeping, because Phase 0 is about getting a physical part in hand, not about
prototyping Construction A. It is throwaway. The connector dimensions it
encodes are not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from geom import MeshData, Pt2, box, merge, prism_yz, rotation_y, translation

UNION = "UNION"
DIFFERENCE = "DIFFERENCE"

Part = tuple[str, MeshData, str]  # (label, mesh, op)
Aabb = tuple[str, tuple[float, float, float], tuple[float, float, float]]


@dataclass(frozen=True)
class Config:
    """Body dimensions are v1's measured values. Connector values are §6.4."""

    # body, §2
    width_outer: float = 24.0
    rail_thickness: float = 1.2
    rail_height_total: float = 4.7
    deck_thickness: float = 1.4

    # connector, §6.4
    lap_length: float = 8.0
    # Calibrated in Phase 0 against a printed comb: 0.15 fitted best, 0.10 was
    # too tight. Frozen. Do not guess at this value again.
    fit_clearance: float = 0.15

    # detent, §6.3. The lap is long enough that the tab is a real cantilever
    # spring, so the rib seats by elastic deflection rather than by consuming
    # rigid-body clearance.
    detent_offset: float = 4.0
    detent_height: float = 0.50
    detent_lead_angle: float = 30.0    # degrees from the lap plane, insertion
    detent_return_angle: float = 60.0  # degrees from the lap plane, pull-out

    # coupon only
    body_length: float = 20.0
    tally_depth: float = 0.6
    tally_width: float = 1.0
    tally_pitch: float = 2.0

    # overlap used to keep boolean inputs volumetrically overlapping rather
    # than face-to-face, which is what generated v1's non-manifold results
    eps: float = 0.01

    # -- derived ---------------------------------------------------------

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
    def half_deck(self) -> float:
        return self.deck_thickness / 2.0

    @property
    def lap_face_z(self) -> float:
        """Height of the (+x) lap face. The (-x) lap face is its negation."""
        return self.fit_clearance / 2.0

    @property
    def centre_gap(self) -> float:
        """Half-width of the slot along x = 0.

        The split runs through x = 0, so in the lap zone our (+x) half slides
        past the mating piece's (-x) half. They need clearance, and the result
        is a narrow slot down the centreline for the length of the lap.
        """
        return self.fit_clearance / 2.0

    @property
    def notch_back(self) -> float:
        """How far the notch is cut back from the port plane, §6.2."""
        return -(self.lap_length + self.fit_clearance)

    @property
    def groove_depth(self) -> float:
        return self.detent_height + self.fit_clearance / 2.0

    def validate(self) -> None:
        for name in ("width_outer", "rail_thickness", "rail_height_total",
                     "deck_thickness", "lap_length", "fit_clearance",
                     "detent_offset", "detent_height", "body_length"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.rail_thickness * 2 >= self.width_outer:
            raise ValueError("rails too thick for the track width")
        if self.deck_thickness >= self.rail_height_total:
            raise ValueError("deck must be thinner than the rail height")
        for name in ("detent_lead_angle", "detent_return_angle"):
            if not 5.0 < getattr(self, name) < 89.0:
                raise ValueError(f"{name} must be between 5 and 89 degrees")
        if self.detent_lead_angle >= self.detent_return_angle:
            raise ValueError(
                "lead-in must be shallower than the return face, or the joint "
                "is as hard to assemble as it is to pull apart"
            )

        rib_lead, rib_return = self._runs(self.detent_height, 0.0)
        if self.detent_offset + rib_lead >= self.lap_length:
            raise ValueError("detent rib would overhang the end of the tab")
        groove_lead, groove_return = self._runs(self.groove_depth,
                                                self.fit_clearance / 2.0)
        if self.detent_offset + groove_lead >= -self.notch_back:
            raise ValueError("detent groove would run past the back of the notch")
        if groove_return >= self.detent_offset - rib_return:
            raise ValueError("detent rib and groove would run into each other")
        if self.lap_face_z + self.groove_depth >= self.half_height:
            raise ValueError("detent groove would cut through the rail")
        if self.centre_gap * 2 >= self.rail_inner:
            raise ValueError("centreline slot is wider than the deck")
        if self.body_length <= self.lap_length + self.fit_clearance:
            raise ValueError("coupon body is shorter than its own notch")

    def _runs(self, height: float, grow: float) -> tuple[float, float]:
        lead = height / math.tan(math.radians(self.detent_lead_angle)) + grow
        ret = height / math.tan(math.radians(self.detent_return_angle)) + grow
        return lead, ret


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------


def _detent_polygon(cfg: Config, y_centre: float, z_face: float, out_dir: int,
                    height: float, grow: float, mirror: bool) -> list[Pt2]:
    """Asymmetric detent profile as a (y, z) polygon.

    Shallow on the insertion side so the joint pushes together, steep on the
    pull-out side so it resists coming apart. ``out_dir`` is +1 when the feature
    points toward +Z and -1 toward -Z. ``mirror`` flips it in y, which is what
    the mating piece's rotation does to it.
    """
    lead_run, return_run = cfg._runs(height, grow)
    base_z = z_face - out_dir * cfg.eps
    apex = (y_centre, z_face + out_dir * height)
    poly = [
        (y_centre + lead_run, base_z),
        apex,
        (y_centre - return_run, base_z),
    ]
    if mirror:
        poly = [(2.0 * y_centre - y, z) for (y, z) in poly]

    area = 0.0
    for i in range(len(poly)):
        ay, az = poly[i]
        by, bz = poly[(i + 1) % len(poly)]
        area += ay * bz - by * az
    return poly if area > 0 else list(reversed(poly))


def coupon_parts(cfg: Config, tally: int = 0) -> list[Part]:
    """Boolean programme for one coupon.

    The port plane is y = 0 and the body runs back to y = -body_length. Tabs
    protrude to y = +lap_length. ``tally`` engraves that many slots in the far
    end face so a printed part can still be identified once it is off the bed.
    """
    cfg.validate()

    hw = cfg.half_width
    ri = cfg.rail_inner
    hh = cfg.half_height
    hd = cfg.half_deck
    L = cfg.lap_length
    c = cfg.fit_clearance
    zf = cfg.lap_face_z
    xs = cfg.centre_gap
    back = cfg.notch_back
    bl = cfg.body_length
    eps = cfg.eps

    parts: list[Part] = []

    # -- body: full section, no split -------------------------------------
    # The deck runs the full width so it overlaps both rails by volume rather
    # than meeting them face to face.
    parts.append(("body_deck", box((-hw, -bl, -hd), (hw, back, hd)), UNION))
    parts.append(("body_rail_px", box((ri, -bl, -hh), (hw, back, hh)), UNION))
    parts.append(("body_rail_nx", box((-hw, -bl, -hh), (-ri, back, hh)), UNION))

    # -- the diagonal split, §6.1 -----------------------------------------
    # Through the lap zone we keep only the (+x, +z) and (-x, -z) quadrants,
    # and they run on past the port plane to y = +L as the tabs. Everything
    # else in the lap zone is empty, and that empty volume is exactly what the
    # mating piece fills.
    parts.append(("tab_rail_px",
                  box((ri, back - eps, zf), (hw, L, hh)), UNION))
    parts.append(("tab_deck_px",
                  box((xs, back - eps, zf), (ri + eps, L, hd)), UNION))
    parts.append(("tab_rail_nx",
                  box((-hw, back - eps, -hh), (-ri, L, -zf)), UNION))
    parts.append(("tab_deck_nx",
                  box((-ri - eps, back - eps, -hd), (-xs, L, -zf)), UNION))

    # -- detents, §6.3 -----------------------------------------------------
    # Rib at +detent_offset, groove at -detent_offset, on each lap face. The
    # longitudinal offset is what stops a part's own rib meeting its partner's.
    # Both sit on the rail, where there is 2.35 mm of depth to work with; the
    # deck tongue is only half the deck thick and has none.
    d = cfg.detent_offset
    dh = cfg.detent_height
    gd = cfg.groove_depth
    grow = c / 2.0

    parts.append((
        "rib_px",
        prism_yz(_detent_polygon(cfg, +d, zf, -1, dh, 0.0, mirror=False),
                 ri + c, hw),
        UNION,
    ))
    parts.append((
        "rib_nx",
        prism_yz(_detent_polygon(cfg, +d, -zf, +1, dh, 0.0, mirror=False),
                 -hw, -ri - c),
        UNION,
    ))
    # Grooves are the mirror of the rib, because the piece that fills them
    # arrives rotated 180 degrees about Z, which flips y.
    parts.append((
        "groove_px",
        prism_yz(_detent_polygon(cfg, -d, zf, +1, gd, grow, mirror=True),
                 ri + c - grow, hw + eps),
        DIFFERENCE,
    ))
    parts.append((
        "groove_nx",
        prism_yz(_detent_polygon(cfg, -d, -zf, -1, gd, grow, mirror=True),
                 -hw - eps, -ri - c + grow),
        DIFFERENCE,
    ))

    # -- tally slots -------------------------------------------------------
    # Cut clean through the deck thickness so they read from either face and
    # cannot break flip symmetry. Centred on x = 0 so they also survive the
    # 180-degree rotation that mates two coupons.
    for i in range(tally):
        cx = (i - (tally - 1) / 2.0) * cfg.tally_pitch
        half = cfg.tally_width / 2.0
        if abs(cx) + half >= ri:
            raise ValueError("tally slots do not fit across the deck")
        parts.append((
            f"tally_{i}",
            box((cx - half, -bl - eps, -hd - eps),
                (cx + half, -bl + cfg.tally_depth, hd + eps)),
            DIFFERENCE,
        ))

    return parts


def solid_aabbs(cfg: Config) -> list[Aabb]:
    """Axis-aligned bounds of the coupon's solid boxes, for the mating test.

    Detent ribs are excluded; they are checked separately in §6.3 terms.
    """
    return [
        (label, tuple(mesh.verts.min(axis=0)), tuple(mesh.verts.max(axis=0)))
        for label, mesh, op in coupon_parts(cfg)
        if op == UNION and not label.startswith("rib_")
    ]


# --------------------------------------------------------------------------
# plate layout
# --------------------------------------------------------------------------


def orientation_matrix(cfg: Config, orientation: str) -> np.ndarray:
    """Transform from model space into print space.

    ``side``  rests one rail's outer face on the bed. Nothing bridges, and the
              21.6 mm channel never spans open air. Tall and narrow, so use a
              brim.
    ``flat``  rests on the two lower rails. Stable, but the deck underside then
              bridges 21.6 mm, and on a flip-symmetric track that underside is
              a driving surface.
    ``model`` leaves the part in spec coordinates.
    """
    if orientation == "model":
        return np.eye(4)
    if orientation == "flat":
        return translation(0.0, 0.0, cfg.half_height)
    if orientation == "side":
        # rotate -90 deg about Y, mapping x -> z, so the x = -half_width face
        # becomes the bed.
        return translation(0.0, 0.0, cfg.half_width) @ rotation_y(-np.pi / 2)
    raise ValueError(f"unknown orientation {orientation!r}")


def plate_offsets(cfg: Config, orientation: str, count: int,
                  gap: float = 8.0) -> list[np.ndarray]:
    """Side-by-side placements for ``count`` identical coupons."""
    if orientation == "side":
        pitch = cfg.rail_height_total + gap
    else:
        pitch = cfg.width_outer + gap
    span = pitch * (count - 1)
    return [translation(i * pitch - span / 2.0, 0.0, 0.0) for i in range(count)]


def comb_values() -> list[float]:
    """The fit_clearance values Phase 0 prints, §10."""
    return [0.10, 0.15, 0.20, 0.25, 0.30]


def preview_mesh(cfg: Config, tally: int = 0) -> MeshData:
    """Union-free concatenation of the coupon's parts.

    Useful for eyeballing placement without Blender. The result is NOT a valid
    solid and must never be validated or printed.
    """
    return merge([mesh for _, mesh, _ in coupon_parts(cfg, tally)])
