import traceback

import adsk.core  # type: ignore
import adsk.fusion  # type: ignore

from .shared_vars import SharedVars


def extractCoords(entity):
    """
    This function is needed because the way to extract the centerpoint is dependent on
    the entity type in fusion.
    """
    center_point = None

    if isinstance(entity, adsk.fusion.SketchPoint):
        center_point = entity.worldGeometry

    elif isinstance(entity, (adsk.fusion.ConstructionPoint, adsk.fusion.BRepVertex)):
        center_point = entity.geometry

    elif isinstance(entity, adsk.fusion.BRepEdge):
        geom = entity.geometry
        center_point = geom.center

    return center_point

def addRowToTable(tableInput: adsk.core.TableCommandInput, rowNumber) -> None:
    """
    Add a row to the table
    """
    # Get the CommandInputs object associated with the parent command.
    cmdInputs = adsk.core.CommandInputs.cast(tableInput.commandInputs)

    # Select type of pulley (toothed or smooth) input
    dropdown_id = f"table_dropdown_{rowNumber}"
    dropdownInput = cmdInputs.addDropDownCommandInput(
        dropdown_id, "Dropdown", adsk.core.DropDownStyles.LabeledIconDropDownStyle
    )

    dropdownItems = dropdownInput.listItems
    dropdownItems.add("Toothed", True, "")
    dropdownItems.add("Smooth", False, "")

    # Diameter/Tooth count input
    value_id = f"table_value_{rowNumber}"
    valueInput = cmdInputs.addValueInput(value_id, "Diameter/Tooth count", "", adsk.core.ValueInput.createByReal(0.0))

    # For the position
    selection_display_id = f"table_selection_display_{rowNumber}"
    selectionDisplayInput = cmdInputs.addStringValueInput(selection_display_id, "Position", "No selection")
    selectionDisplayInput.isReadOnly = True

    # Add the inputs to the table.
    row = tableInput.rowCount
    tableInput.addCommandInput(dropdownInput, row, 0)
    tableInput.addCommandInput(valueInput, row, 1)
    tableInput.addCommandInput(selectionDisplayInput, row, 2)  # Add the string input instead


class CommandInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self, shared_vars: SharedVars) -> None:
        super().__init__()
        self.shared_vars = shared_vars

    def notify(self, args: object) -> None:
        try:
            app = adsk.core.Application.get()
            ui = app.userInterface
            eventArgs = adsk.core.InputChangedEventArgs.cast(args)
            inputs = eventArgs.inputs
            cmdInput = eventArgs.input

            tableInput = inputs.itemById("table")
            selected_row = tableInput.selectedRow

            if cmdInput.id == "master_selection":
                masterSelection = adsk.core.SelectionCommandInput.cast(cmdInput)

                if selected_row == -1:
                    if ui:
                        ui.messageBox("Please select a row in the table first!")
                    masterSelection.clearSelection()
                    return

                row_string_input = tableInput.getInputAtPosition(tableInput.selectedRow, 2)

                if masterSelection.selectionCount > 0:
                    selected_entity = masterSelection.selection(0).entity

                    # Add coordinate of selection to raw center points
                    center_coord = extractCoords(selected_entity)
                    self.shared_vars.raw_center_points[selected_row] = center_coord

                    entity_name = "Selected Object"
                    if hasattr(selected_entity, "name"):
                        entity_name = selected_entity.name
                    elif hasattr(selected_entity, "body"):
                        entity_name = "Edge/Vertex on: " + selected_entity.body.name
                    elif hasattr(selected_entity, "parentSketch"):
                        entity_name = "Point in: " + selected_entity.parentSketch.name

                    row_string_input.value = entity_name

                    masterSelection.clearSelection()
                else:
                    row_string_input.value = "No selection"

            # Add / delete rows
            tableInput = inputs.itemById("table")
            if cmdInput.id == "tableAdd":
                addRowToTable(tableInput, self.shared_vars.rowNumber)
                
                # Increment a counter used to make each row unique.
                self.shared_vars.rowNumber += 1
                self.shared_vars.raw_center_points.append(None)
            elif cmdInput.id == "tableDelete":
                if selected_row == -1:
                    ui.messageBox("Select one row to delete.")
                else:
                    tableInput.deleteRow(selected_row)
                    self.shared_vars.raw_center_points.pop(selected_row)  # Remove coordinate

            # Enforce spinner limit
            max_idx = max(0, tableInput.rowCount - 1)
            spinner = inputs.itemById("tensioner_idx")

            if cmdInput.id == "tableDelete" and spinner.value > max_idx:
                spinner.value = max_idx

            if cmdInput.id == "tensioner_idx" and spinner.value > max_idx:
                spinner.value = max_idx

        except:  # noqa: E722
            ui.messageBox(f"Failed:\n{traceback.format_exc()}")


