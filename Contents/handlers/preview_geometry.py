"""
Geometry and belt-building helpers used by the execute-preview handler.
"""

import math
from collections.abc import Callable

import adsk.core  # type: ignore
import adsk.fusion  # type: ignore

from belt_geometry import Belt, BeltFace, BeltProfile, Circle, Vec2

from . import config


def project_points(
    raw_center_points: list[adsk.core.Point3D],
    sketch: adsk.fusion.Sketch,
    tensioner_point3d: adsk.core.Point3D,
) -> tuple[list[Vec2], Vec2] | tuple[None, None]:
    """
    Project the raw selected points and tensioner point into sketch space.
    """

    projected_points: list[Vec2] = []
    for raw_point in raw_center_points:
        if raw_point is not None:
            projected_point = sketch.modelToSketchSpace(raw_point)
            projected_points.append(Vec2([projected_point.x, projected_point.y]))
        else:
            return None, None

    projected_tensioner_point = sketch.modelToSketchSpace(tensioner_point3d)
    return projected_points, Vec2(projected_tensioner_point.x, projected_tensioner_point.y)


def parse_table_input(
    lam: float,
    table_input: adsk.core.TableCommandInput,
    tensioner_idx: int,
    projected_points: list[Vec2],
    projected_tensioner_point: Vec2,
    pitch: float,
) -> tuple[list[Circle], list[BeltFace]] | bool:
    """
    Parse table inputs into midline pulley circles and belt topology.

    The topology is also used to create the backing and tooth-tip offset belts.
    """

    if projected_points is None or projected_tensioner_point is None:
        return False

    midline_pulleys = []
    topology = []
    to_mm = config.to_mm
    for index, point in enumerate(projected_points):
        type_input = table_input.getInputAtPosition(index, 0)
        if type_input.selectedItem.name == "Toothed":
            tooth_count = table_input.getInputAtPosition(index, 1).value
            diameter_value = tooth_count * pitch / math.pi
            topology.append(BeltFace.FRONT)
        else:
            diameter_value = table_input.getInputAtPosition(index, 1).value
            topology.append(BeltFace.BACK)

        radius = diameter_value / 2
        coords = (point.x * to_mm, point.y * to_mm)
        if index == tensioner_idx:
            coords = Vec2(coords)
            coords += lam * (projected_tensioner_point * to_mm - coords)
            coords = coords.to_list()
        midline_pulleys.append(Circle(radius, coords, name=f"Pulley {index}"))

    return midline_pulleys, topology


def compute_target_length(
    belt: Belt,
    target_length: float,
    tensioner_idx: int,
    projected_points: list[Vec2],
    projected_tensioner_point: Vec2,
    to_mm: float,
) -> float | bool:
    """
    Compute the slide-vector position needed to reach a target belt length.

    Uses the belt solver's secant method because the search space can be
    discontinuous and a bisection method is not reliable for every configuration.

    Returns:
        The normalized position along the tensioner slide vector, or ``False``
        when the solver cannot find a position.
    """

    tensioner_pulley_position = projected_points[tensioner_idx] * to_mm
    projected_tensioner_point = projected_tensioner_point * to_mm
    slide_vector = projected_tensioner_point - tensioner_pulley_position

    if not belt.find_movable_circle_position(
        target_length, tensioner_idx, slide_vector, method="secant"
    ):
        return False

    current_circle = belt.circles[tensioner_idx]
    diff = Vec2(current_circle.x, current_circle.y) - tensioner_pulley_position
    norm_squared = slide_vector.norm() ** 2
    if norm_squared == 0:
        return 0.0

    return slide_vector.dot(diff) / norm_squared


def adjust_tensioner_position(
    midline_belt: Belt,
    tensioner_idx: int,
    projected_points: list[Vec2],
    projected_tensioner_point: Vec2,
    sketch: adsk.fusion.Sketch,
    error_message: Callable[[str, adsk.fusion.Sketch], None],
    profile: BeltProfile
) -> bool:
    """
    Adjust the tensioner position so the belt length is evenly divisible by the pitch.

    The function attempts to round the length up first and then down. If neither
    target can be reached on the tensioner's slide path, it reports the error
    through ``error_message``.

    Returns:
        ``True`` when a valid adjustment was found, otherwise ``False``.
    """

    tensioner_circle = midline_belt.circles[tensioner_idx]
    original_coords = tensioner_circle.coords
    to_mm = config.to_mm

    def restore_original_position() -> None:
        tensioner_circle.coords = original_coords
        tensioner_circle.x, tensioner_circle.y = original_coords
        midline_belt._calculate_geometry()

    initial_belt_length = midline_belt.total_length
    target_length = math.ceil(initial_belt_length / profile.pitch) * profile.pitch
    lam_rev = compute_target_length(
        midline_belt,
        target_length,
        tensioner_idx,
        projected_points,
        projected_tensioner_point,
        to_mm,
    )

    if lam_rev is False:
        restore_original_position()
        return False

    tolerance = 1e-3
    if not (-tolerance <= lam_rev <= 1.0 + tolerance):
        restore_original_position()
        target_length = math.floor(initial_belt_length / profile.pitch) * profile.pitch
        lam_rev = compute_target_length(
            midline_belt,
            target_length,
            tensioner_idx,
            projected_points,
            projected_tensioner_point,
            to_mm,
        )
        if not (-tolerance <= lam_rev <= 1.0 + tolerance):
            restore_original_position()
            error_message(
                f"The solver failed or the tensioner path is too short! Try a different configuration. Lam_rev = {lam_rev}",
                sketch,
            )
            return False

    return True


def create_offset_pulleys(
    midline_pulleys: list[Circle],
    topology: list[BeltFace],
    back_center: float,
    tip_end: float,
) -> tuple[list[Circle], list[Circle]]:
    """Create the backing and tooth-tip pulley traces for preview generation.

    Front/toothed pulleys expand toward the backing and contract toward the tip;
    smooth/back pulleys apply the inverse offsets.
    """

    backing_pulleys = []
    tip_pulleys = []
    for index, pulley in enumerate(midline_pulleys):
        backing_pulley = Circle(pulley.r, pulley.coords, name=pulley.name)
        tip_pulley = Circle(pulley.r, pulley.coords, name=pulley.name)

        if topology[index] == BeltFace.FRONT:
            backing_pulley.r += back_center
            tip_pulley.r -= tip_end
        else:
            backing_pulley.r -= back_center
            tip_pulley.r += tip_end

        backing_pulleys.append(backing_pulley)
        tip_pulleys.append(tip_pulley)

    return backing_pulleys, tip_pulleys
