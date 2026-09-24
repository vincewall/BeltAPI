"""
Custom graphics helpers used by the execute-preview handler.
"""

import adsk.core  # type: ignore
import adsk.fusion  # type: ignore

from belt_geometry.replacements import Vec2
from belt_geometry.solver import Belt, Circle

from . import config


def make_graphic_preview(
    belt: Belt,
    color: adsk.core.Color,
    sketch: adsk.fusion.Sketch,
    preview_graphics: adsk.fusion.CustomGraphicsGroup,
) -> None:
    """
    Generate custom graphics lines and arcs for a belt preview.
    """
    to_mm = config.to_mm

    line_points = []
    for line in belt.lines:
        start = adsk.core.Point3D.create(line[0].x / to_mm, line[0].y / to_mm, 0)
        end = adsk.core.Point3D.create(line[1].x / to_mm, line[1].y / to_mm, 0)
        line_points.extend([sketch.sketchToModelSpace(start), sketch.sketchToModelSpace(end)])

    coordinates = []
    for point in line_points:
        coordinates.extend([point.x, point.y, point.z])

    if coordinates:
        graphics_coordinates = adsk.fusion.CustomGraphicsCoordinates.create(coordinates)
        custom_lines = preview_graphics.addLines(graphics_coordinates, [], False)
        custom_lines.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(color)
        custom_lines.weight = 2

    for arc in belt.arcs:
        mid = len(arc) // 2
        points = []
        for index in [0, mid, -1]:
            p = sketch.sketchToModelSpace(adsk.core.Point3D.create(arc[index].x / to_mm, arc[index].y / to_mm, 0))
            points.append(p)

        arc3d = adsk.core.Arc3D.createByThreePoints(*points)
        custom_arc = preview_graphics.addCurve(arc3d)
        custom_arc.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(color)
        custom_arc.weight = 2


def tensioner_vector_graphic_preview(
    midline_pulleys: list[Circle],
    projected_points: list[Vec2],
    projected_tensioner_point: Vec2,
    tensioner_idx: int,
    sketch: adsk.fusion.Sketch,
    preview_graphics: adsk.fusion.CustomGraphicsGroup,
    color: adsk.core.Color,
) -> None:
    """
    Draw the tensioner slide vector and movable pulley center marker (img).
    """
    to_mm = config.to_mm
    crosshair_img_dir = config.crosshair_img_dir

    pulley = midline_pulleys[tensioner_idx]
    pulley_point = sketch.sketchToModelSpace(adsk.core.Point3D.create(pulley.x / to_mm, pulley.y / to_mm, 0))
    point_coordinates = adsk.fusion.CustomGraphicsCoordinates.create([pulley_point.x, pulley_point.y, pulley_point.z])
    point_style = adsk.fusion.CustomGraphicsPointTypes.UserDefinedCustomGraphicsPointType
    preview_graphics.addPointSet(point_coordinates, [], point_style, crosshair_img_dir)



    start = pulley_point
    end = sketch.sketchToModelSpace(adsk.core.Point3D.create(*projected_tensioner_point.to_list(), 0))
    coordinates = []
    for point in [start, end]:
        coordinates.extend([point.x, point.y, point.z])

    graphics_coordinates = adsk.fusion.CustomGraphicsCoordinates.create(coordinates)
    tensioner_line = preview_graphics.addLines(graphics_coordinates, [], False)
    tensioner_line.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(color)
    tensioner_line.weight = 2


def place_billboarded_text(
    preview_graphics: adsk.fusion.CustomGraphicsGroup,
    text: str,
    position: tuple[float, float, float],
    size: float,
    color: adsk.core.Color,
    sketch: adsk.fusion.Sketch = None,
) -> None:
    """
    Place billboarded text at the specified coordinate.

    If a sketch is supplied, ``position`` is interpreted in sketch space;
    otherwise it is interpreted directly in model space.
    """

    x, y, z = position
    anchor_point = adsk.core.Point3D.create(x, y, z)
    if sketch:
        anchor_point = sketch.sketchToModelSpace(anchor_point)

    transform = adsk.core.Matrix3D.create()
    transform.translation = anchor_point.asVector()
    placed_text = preview_graphics.addText(text, "Arial", 1.0, transform)
    placed_text.billBoarding = adsk.fusion.CustomGraphicsBillBoard.create(adsk.core.Point3D.create(0, 0, 0))
    local_anchor = adsk.core.Point3D.create(0, 0, 0)
    placed_text.viewScale = adsk.fusion.CustomGraphicsViewScale.create(size, local_anchor)
    placed_text.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(color)


def place_length_text(
    preview_graphics: adsk.fusion.CustomGraphicsGroup,
    projected_points: list[Vec2],
    midline_pulleys: list[Circle],
    midline_belt: Belt,
    sketch: adsk.fusion.Sketch,
    length_text_color: adsk.core.Color,
) -> None:
    """
    Display the midline belt's total length in the lower right area of the preview.
    """
    to_mm = config.to_mm

    valid_points = [point for point in projected_points if point is not None]
    max_x = max((point.x for point in valid_points), default=0.0)
    min_y = min((point.y for point in valid_points), default=0.0)
    max_radius = max((pulley.r for pulley in midline_pulleys), default=0.0)
    position = ((max_x + 2 * max_radius) / to_mm, (min_y + 2 * max_radius) / to_mm, 0.0)
    place_billboarded_text(
        preview_graphics,
        f"Total Length: {midline_belt.total_length:.2f} mm",
        position,
        24,
        length_text_color,
        sketch,
    )


def error_message(
    preview_graphics: adsk.fusion.CustomGraphicsGroup,
    solver_error: Exception | str,
    raw_center_points: list[adsk.core.Point3D],
    sketch: adsk.fusion.Sketch,
    red: adsk.core.Color,
) -> None:
    """
    Display an invalid-path message near the center of the pulley system.
    """

    valid_points = [point for point in raw_center_points if point is not None]
    count = len(valid_points)
    if count:
        position = (
            sum(point.x for point in valid_points) / count,
            sum(point.y for point in valid_points) / count,
            sum(point.z for point in valid_points) / count,
        )
    else:
        position = (0.0, 0.0, 0.0)

    place_billboarded_text(
        preview_graphics,
        f"Invalid Path: {solver_error!s}",
        position,
        30,
        red,
        sketch,
    )
