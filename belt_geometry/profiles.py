"""
Contains all tooth profiles.
"""

from dataclasses import dataclass, field
from enum import IntEnum

#from typing import ClassVar
from .replacements import Vec2


class Pt(IntEnum):
    """Indices for the tooth profile points"""

    BACK_CENTER = 0
    BACK_LOWER_CORNER = 1
    FLAT_UPPER_CORNER = 2
    ROOT_FILL_START = 3
    FLANK_FILL_START = 4
    TIP_FILL_START = 5
    TIP_END = 6


class Ctr(IntEnum):
    """Indices for the circle centers"""

    TIP = 0
    FLANK = 1
    ROOT = 2


@dataclass
class BeltProfile:
    """A data container for belt dimensions."""

    name: str
    pitch: float
    pld: float
    base_points_left: list[Vec2]
    base_centers_left: list[Vec2]

    r_root: float = field(init=False)
    r_flank: float = field(init=False)
    r_tip: float = field(init=False)

    all_profiles_dict = {}  # noqa: RUF012

    def __post_init__(self) -> None:
        """Compute all radiuses from the given points assuming tangencies."""
        self.r_root = (
            self.base_centers_left[Ctr.ROOT] - self.base_points_left[Pt.ROOT_FILL_START]
        ).norm()
        self.r_flank = (
            self.base_centers_left[Ctr.FLANK]
            - self.base_points_left[Pt.FLANK_FILL_START]
        ).norm()
        self.r_tip = (
            self.base_centers_left[Ctr.TIP] - self.base_points_left[Pt.TIP_FILL_START]
        ).norm()

        self.all_profiles_dict[self.name] = self

    @property
    def back_center(self) -> float:
        return abs(self.base_points_left[Pt.BACK_CENTER].y)

    @property
    def tip_end(self) -> float:
        return abs(self.base_points_left[Pt.TIP_END].y)

# Example profile for GT2 2mm pitch belt. All dimensions are in mm.
GT2_2MM = BeltProfile(
    name="GT2 2mm",
    pitch=2.0,
    pld=0.254,
    base_points_left=[
        Vec2([0.0, -0.376]),  # Pt.BACK_CENTER
        Vec2([-1.0, -0.376]),  # Pt.BACK_LOWER_CORNER
        Vec2([-1.0, 0.254]),  # Pt.FLAT_UPPER_CORNER
        Vec2([-0.74017546, 0.254]),  # Pt.ROOT_FILL_START
        Vec2([-0.59145712, 0.38443331]),  # Pt.FLANK_FILL_START
        Vec2([-0.49887711, 0.69220106]),  # Pt.TIP_FILL_START
        Vec2([0.0, 1.004]),  # Pt.TIP_END
    ],
    base_centers_left=[
        Vec2([0.0, 0.449]),  # Ctr.TIP
        Vec2([0.4, 0.254]),  # Ctr.FLANK
        Vec2([-0.74017546, 0.404]),  # Ctr.ROOT
    ],
)

# Example profile for GT3 3mm pitch belt. All dimensions are in mm.
GT3_3MM = BeltProfile(
    name="GT3 3mm",
    pitch=3.0,
    pld=0.381,
    base_points_left=[
        Vec2([0.0, -0.879]),  # Pt.BACK_CENTER
        Vec2([-1.5, -0.879]),  # Pt.BACK_LOWER_CORNER
        Vec2([-1.5, 0.381]),  # Pt.FLAT_UPPER_CORNER
        Vec2([-1.14404397, 0.381]),  # Pt.ROOT_FILL_START
        Vec2([-0.89629765, 0.59750733]),  # Pt.FLANK_FILL_START
        Vec2([-0.7738806, 1.02258046]),  # Pt.TIP_FILL_START
        Vec2([0.0, 1.521]),  # Pt.TIP_END
    ],
    base_centers_left=[
        Vec2([0.0, 0.671]),  # Ctr.TIP
        Vec2([0.61, 0.39387187]),  # Ctr.FLANK
        Vec2([-1.14404397, 0.631]),  # Ctr.ROOT
    ],
)