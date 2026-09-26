"""
Creates the belt in fusion.

The execute handler creates the belt in fusion using extrusions from sketches, pattern
on bath for straight segments and circular patterns for curved segments.
"""

import math
import traceback

import adsk.core  # type: ignore

from belt_geometry import BeltSystem

from . import config
from .shared_vars import SharedVars


def extrude_tooth(comp, sketch, thickness):
    # Create an extrusion.
    extInput = comp.features.extrudeFeatures.createInput(
        sketch.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    distance = adsk.core.ValueInput.createByReal(thickness / 2)
    extInput.setDistanceExtent(True, distance)
    ext = comp.features.extrudeFeatures.add(extInput)

    return ext


def pattern_along_path(comp, ext, path_in_sketch, num_teeth, belt):

    # Create input entities for the path pattern.
    inputEntities = adsk.core.ObjectCollection.create()
    body = ext.bodies.item(0)
    inputEntities.add(body)

    # Create path for path pattern
    features = comp.features
    path = features.createPath(path_in_sketch)

    # Quantity and distance
    quantity = adsk.core.ValueInput.createByReal(num_teeth)
    patternDistance = adsk.core.ValueInput.createByReal(belt.profile.pitch / config.to_mm)

    # Create the input for path pattern
    pathPatterns = features.pathPatternFeatures
    pathPatternInput = pathPatterns.createInput(
        inputEntities, path, quantity, patternDistance, adsk.fusion.PatternDistanceType.SpacingPatternDistanceType
    )
    pathPatternInput.isOrientationAlongPath = True

    # Create the path pattern
    _ = pathPatterns.add(pathPatternInput)


def pattern_straight_seg(start_tooth, end_tooth, comp, sketch, ext, num_teeth, belt):
    # Create a sketch line to pattern along
    start_position = start_tooth.position
    end_position = end_tooth.position
    to_mm = config.to_mm

    startPoint = adsk.core.Point3D.create(start_position.x / to_mm, start_position.y / to_mm, 0)
    endPoint = adsk.core.Point3D.create(end_position.x / to_mm, end_position.y / to_mm, 0)

    lines = sketch.sketchCurves.sketchLines
    pathLine = lines.addByTwoPoints(startPoint, endPoint)

    pattern_along_path(comp, ext, pathLine, num_teeth, belt)


def pattern_curved_seg(start_tooth, mid_tooth, end_tooth, angle_expression, quantity, comp, sketch, ext):
    # Create a sketch line to pattern along
    start_position = start_tooth.position
    mid_position = mid_tooth.position
    end_position = end_tooth.position
    to_mm = config.to_mm

    startPoint = adsk.core.Point3D.create(start_position.x / to_mm, start_position.y / to_mm, 0)
    midPoint = adsk.core.Point3D.create(mid_position.x / to_mm, mid_position.y / to_mm, 0)
    endPoint = adsk.core.Point3D.create(end_position.x / to_mm, end_position.y / to_mm, 0)

    arcs = sketch.sketchCurves.sketchArcs
    pathArc = arcs.addByThreePoints(startPoint, midPoint, endPoint)

    # Create the input for circular pattern
    input_entities = adsk.core.ObjectCollection.create()
    input_entities.add(ext.bodies.item(0))
    circularFeats = comp.features.circularPatternFeatures
    circularFeatInput = circularFeats.createInput(input_entities, pathArc)

    # Set the quantity of the elements
    circularFeatInput.quantity = adsk.core.ValueInput.createByReal(quantity)

    # Set the angle of the circular pattern
    circularFeatInput.totalAngle = adsk.core.ValueInput.createByString(angle_expression)

    # Set symmetry of the circular pattern
    circularFeatInput.isSymmetric = False

    # Create the circular pattern
    _ = circularFeats.add(circularFeatInput)


def create_new_comp(rootComp):
    # Create a new occurrence.
    trans = adsk.core.Matrix3D.create()
    occ = rootComp.occurrences.addNewComponent(trans)

    # Get the associated component.
    newComp = occ.component
    return newComp


def draw_tooth(comp, tooth, selected_plane_entity):
    """
    Draws a tooth in a new sketch inside the specified component.
    """
    sketches = comp.sketches
    sketch = sketches.add(selected_plane_entity)
    to_mm = config.to_mm

    lines = sketch.sketchCurves.sketchLines
    arcs = sketch.sketchCurves.sketchArcs
    all_straight_segs = []
    all_straight_segs.extend(tooth._create_all_straight_line_segments(side="left"))
    all_straight_segs.extend(tooth._create_all_straight_line_segments(side="right"))

    for segment in all_straight_segs:
        p0 = segment[0]
        p1 = segment[1]

        point1 = adsk.core.Point3D.create(p0.x / to_mm, p0.y / to_mm, 0)
        point2 = adsk.core.Point3D.create(p1.x / to_mm, p1.y / to_mm, 0)
        lines.addByTwoPoints(point1, point2)

    # Draw arcs

    all_arc_segs = []
    all_arc_segs.extend(tooth._create_all_arcs(side="left"))
    all_arc_segs.extend(tooth._create_all_arcs(side="right"))

    for segment in all_arc_segs:
        p0 = segment[0]
        p1 = segment[len(segment) // 2]
        p2 = segment[-1]

        # Check that points are not in the same spot
        tol = 1e-6
        dist = math.hypot(p0.x - p2.x, p0.y - p2.y)

        if dist < tol:
            continue

        point1 = adsk.core.Point3D.create(p0.x / to_mm, p0.y / to_mm, 0)
        point2 = adsk.core.Point3D.create(p1.x / to_mm, p1.y / to_mm, 0)
        point3 = adsk.core.Point3D.create(p2.x / to_mm, p2.y / to_mm, 0)
        arcs.addByThreePoints(point1, point2, point3)

    return sketch


class CommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self, shared_vars: SharedVars) -> None:
        super().__init__()
        self.shared_vars = shared_vars

    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        if not self.shared_vars.valid_midline_belt:
            ui.messageBox("Cannot create belt before establishing a valid midline belt path!")
            return
        try:
            eventArgs = adsk.core.CommandEventArgs.cast(args)
            inputs = eventArgs.command.commandInputs

            belt = BeltSystem(self.shared_vars.valid_midline_belt, self.shared_vars.profile)

            app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(app.activeProduct)

            if not design:
                return

            rootComp = design.rootComponent

            # Create a sketch on plane
            planeInput = inputs.itemById("plane_selection")
            selected_plane_entity = planeInput.selection(0).entity
            thickness_input = inputs.itemById("belt_thickness")
            thickness = thickness_input.value

            # Create and name new component
            comp = create_new_comp(rootComp)
            name = f"{self.shared_vars.profile.name} Belt {self.shared_vars.valid_midline_belt.total_length:.0f}mm"
            comp.name = name

            # Create a group of all features and sketches created
            is_parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
            if is_parametric:
                timeline_start_index = design.timeline.markerPosition

            # Draw, extrude and pattern the "regular" teeth
            for start_idx, end_idx, D in belt.blocks:
                # This would not be a valid block
                if start_idx > end_idx:
                    continue

                start_tooth = belt.teeth[start_idx]
                end_tooth = belt.teeth[end_idx]

                tooth_count = end_idx - start_idx + 1
                if tooth_count <= 2:
                    sketch = draw_tooth(comp, start_tooth, selected_plane_entity)
                    ext = extrude_tooth(comp, sketch, thickness)

                    if tooth_count == 2:
                        sketch = draw_tooth(comp, end_tooth, selected_plane_entity)
                        ext = extrude_tooth(comp, sketch, thickness)
                else:
                    # Pattern the rest of the teeth. First tooth matters
                    if D is None:
                        # Draw and extrude the first tooth
                        sketch = draw_tooth(comp, start_tooth, selected_plane_entity)
                        ext = extrude_tooth(comp, sketch, thickness)

                        # Straight segment
                        num_teeth = int(end_idx - start_idx + 1)
                        pattern_straight_seg(start_tooth, end_tooth, comp, sketch, ext, num_teeth, belt)
                    else:
                        sketch = draw_tooth(comp, start_tooth, selected_plane_entity)
                        ext = extrude_tooth(comp, sketch, thickness)
                        # Curved segment (only work for at least 3 teeth)
                        mid_idx = start_idx + (end_idx - start_idx) // 2
                        mid_tooth = belt.teeth[mid_idx]

                        # This is needed to avoid rounding errors in fusion. Cause is
                        # unknown currently
                        angle_expression = (
                            f"-({belt.profile.pitch!r} * 2 / {D!r}) * {tooth_count - 1} * 180 / PI * 1 deg"
                        )

                        pattern_curved_seg(
                            start_tooth,
                            mid_tooth,
                            end_tooth,
                            angle_expression,
                            tooth_count,
                            comp,
                            sketch,
                            ext,
                        )

            # Draw and extrude transition teeth
            for tooth in [belt.teeth[idx] for idx in belt.transition_idx]:
                sketch = draw_tooth(comp, tooth, selected_plane_entity)
                extrude_tooth(comp, sketch, thickness)

            features = comp.features
            combine_features = features.combineFeatures

            if comp.bRepBodies.count > 1:
                target_body = comp.bRepBodies.item(0)
                tool_bodies = adsk.core.ObjectCollection.create()

                for index in range(1, comp.bRepBodies.count):
                    tool_bodies.add(comp.bRepBodies.item(index))

                combine_input = combine_features.createInput(target_body, tool_bodies)
                combine_input.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
                combine_input.isKeepToolBodies = False
                combine_input.isNewComponent = False

                combine_features.add(combine_input)

            # Finish Create group
            if is_parametric:
                timeline_end_index = design.timeline.markerPosition - 1
            timeline_group = design.timeline.timelineGroups.add(timeline_start_index, timeline_end_index)
            timeline_group.name = "Timing Belt Generator"

        except Exception:  # noqa: BLE001
            ui.messageBox(f"Failed:\n{traceback.format_exc()}")
