"""
The execute preview renders a belt outline using custom graphics. If the generated belt
is deemed valid, the midline is save to the shared variables to be used in the execute.

Possible improvement: Make the script not create a sketch for the projection every time
a preview is rendered.
"""

import traceback
from copy import deepcopy

import adsk.core  # type: ignore
import adsk.fusion  # type: ignore

from belt_geometry.profiles import BeltProfile

from . import config
from .preview_geometry import *
from .preview_graphics import *
from .shared_vars import SharedVars


def has_selection(user_input: adsk.core.SelectionCommandInput) -> bool:
    return user_input is not None and user_input.selectionCount > 0


def check_table_selections(table_input):
    min_diameter = config.min_diameter
    min_tooth_count = config.min_tooth_count
    row_count = table_input.rowCount

    # Cross detection does not work for this case, so catch it here instead
    if row_count == 2:
        type_input_1 = table_input.getInputAtPosition(0, 0)
        type_input_2 = table_input.getInputAtPosition(1, 0)

        if type_input_1.selectedItem.name != type_input_2.selectedItem.name:
            return False

    # Make sure min diameter and tooth count is respected
    for row in range(row_count):
        type_input = table_input.getInputAtPosition(row, 0)
        value_input = table_input.getInputAtPosition(row, 1)

        d_or_num_teeth = value_input.value

        if type_input.selectedItem.name == "Toothed":
            if d_or_num_teeth < min_tooth_count:
                return False
        else:
            if d_or_num_teeth < min_diameter:
                return False

    return True


def extractCoords(entity):
    center_point = None

    if isinstance(entity, adsk.fusion.SketchPoint):
        center_point = entity.worldGeometry

    elif isinstance(entity, (adsk.fusion.ConstructionPoint, adsk.fusion.BRepVertex)):
        center_point = entity.geometry

    elif isinstance(entity, adsk.fusion.BRepEdge):
        geom = entity.geometry
        center_point = geom.center

    return center_point


class CommandExecutePreviewHandler(adsk.core.CommandEventHandler):
    def __init__(self, shared_vars: SharedVars) -> None:
        super().__init__()
        self.shared_vars = shared_vars
        
        self.preview_graphics = shared_vars.preview_graphics
        self.raw_center_points = shared_vars.raw_center_points

    def notify(self, args) -> None:
        """
        Notify update function. TODO: Some error messages may be ambiguous.
        """

        try:
            eventArgs = adsk.core.CommandEventArgs.cast(args)
            inputs = eventArgs.command.commandInputs

            app = adsk.core.Application.get()
            ui = app.userInterface
            design = adsk.fusion.Design.cast(app.activeProduct)

            if not design:
                return
            rootComp = design.rootComponent

            # Create a preview
            des = adsk.fusion.Design.cast(app.activeProduct)
            root = des.rootComponent

            if self.preview_graphics:
                self.preview_graphics.deleteMe()

            self.preview_graphics = root.customGraphicsGroups.add()
            self.shared_vars.preview_graphics = self.preview_graphics

            # Projection
            planeInput = inputs.itemById("plane_selection")
            tensionerInput = inputs.itemById("tensioner_selection")

            profile_dropdown = inputs.itemById("belt_profile")
            selected_profile_name = profile_dropdown.selectedItem.name
            current_profile = BeltProfile.all_profiles_dict[selected_profile_name]
            self.shared_vars.profile = current_profile
            
            # Preview the center points with their indices
            for index, raw_pt in enumerate(self.raw_center_points):
                if raw_pt is None:
                    continue
                place_billboarded_text(
                    self.preview_graphics,
                    text=str(index),
                    position=(raw_pt.x, raw_pt.y, raw_pt.z),
                    size=24,
                    color=config.blue,
                )

            tableInput = inputs.itemById("table")

            if (
                has_selection(planeInput)
                and has_selection(tensionerInput)
                and len(self.raw_center_points) > 1
                and not any(point is None for point in self.raw_center_points)
                and check_table_selections(tableInput)
            ):
                # Create a sketch on plane
                selected_plane_entity = planeInput.selection(0).entity
                sketches = rootComp.sketches
                sketch = sketches.add(selected_plane_entity)

                # Project the raw points onto the sketch plane
                tensioner_point = tensionerInput.selection(0).entity
                tensioner_point3d = extractCoords(tensioner_point)

                # Reverse the order if checkbox is marked
                proj_p, proj_tensioner_p = project_points(self.raw_center_points, sketch, tensioner_point3d)
                
                # Create the belt using the projected points and the specified topology
                tensioner_idx = inputs.itemById("tensioner_idx").value
                lam = inputs.itemById("slider_tensioner").valueOne

                midline_pulleys, topology = parse_table_input(
                    lam,
                    tableInput,
                    tensioner_idx,
                    proj_p,
                    proj_tensioner_p,
                    current_profile.pitch,
                )

                # Reverse the order if checkbox is marked
                if inputs.itemById("reverse").value:
                    midline_pulleys.reverse()
                    topology.reverse()
                    proj_p.reverse()
                    tensioner_idx = len(midline_pulleys) - 1 - tensioner_idx

                # Attempt to create midline belt
                try:
                    midline_belt = Belt(circles=midline_pulleys, allow_crossing=True, topology=topology)

                    is_crossing = midline_belt._check_crossing_lines()

                    if not is_crossing:
                        fallback = deepcopy(midline_belt)

                        success = adjust_tensioner_position(
                            midline_belt,
                            tensioner_idx,
                            proj_p,
                            proj_tensioner_p,
                            sketch,
                            lambda solver_error, error_sketch: error_message(
                                self.preview_graphics,
                                solver_error,
                                self.raw_center_points,
                                error_sketch,
                                config.red,
                            ),
                            current_profile
                        )

                        if not success:
                            midline_belt = fallback

                except Exception as e:  # noqa: BLE001
                    error_message(
                        self.preview_graphics,
                        f"Path Calculation Failed: {e!s}",
                        self.raw_center_points,
                        sketch,
                        config.red,
                    )
                    app.activeViewport.refresh()
                    sketch.deleteMe()
                    return

                self.shared_vars.valid_midline_belt = midline_belt

                # Once the midline has been created successfully, create the backing
                # and Tooth tip trace
                backing_pulleys, tip_pulleys = create_offset_pulleys(
                    midline_pulleys, topology, current_profile.back_center, current_profile.tip_end
                )
                backing_belt = Belt(circles=backing_pulleys, allow_crossing=True, topology=topology)
                tip_belt = Belt(circles=tip_pulleys, allow_crossing=True, topology=topology)

                # Create the graphical preview
                length_text_color = config.green

                # Check for collisions
                for belt in [backing_belt, tip_belt]:
                    collision = belt._check_crossing_lines()
                    if collision:
                        color = config.red
                        length_text_color = config.red
                        self.shared_vars.valid_midline_belt = None
                    else:
                        color = config.green

                    make_graphic_preview(belt, color, sketch, self.preview_graphics)

                # Midline
                make_graphic_preview(midline_belt, config.gray, sketch, self.preview_graphics)

                # Display the tensioner vector and the tensioner pulley position
                tensioner_vector_graphic_preview(
                    midline_pulleys,
                    proj_p,
                    proj_tensioner_p,
                    tensioner_idx,
                    sketch,
                    self.preview_graphics,
                    config.black,
                )

                # Display the total length of the belt in the preview (bottom corner)
                place_length_text(
                    self.preview_graphics,
                    proj_p,
                    midline_pulleys,
                    midline_belt,
                    sketch,
                    length_text_color,
                )

                # Refresh and delete sketch
                app.activeViewport.refresh()
                sketch.deleteMe()

                # Save the valid belt for

        except Exception:  # noqa: BLE001
            ui.messageBox(f"Failed:\n{traceback.format_exc()}")
