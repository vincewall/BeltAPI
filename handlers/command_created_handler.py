"""
Creates the dialog and connects to the other event handlers.

New profiles added in `belt_geometry` will be added automatically to the selection.
"""

import traceback

import adsk.core  # type: ignore

from belt_geometry.profiles import BeltProfile

from . import config
from .command_destroy_handler import CommandDestroyHandler
from .command_execute_handler import CommandExecuteHandler
from .command_execute_preview_handler import CommandExecutePreviewHandler
from .command_input_changed_handler import CommandInputChangedHandler, addRowToTable
from .shared_vars import SharedVars


class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self, shared_vars: SharedVars) -> None:
        super().__init__()
        self.handlers = []
        self.shared_vars = shared_vars

    def notify(self, args: object) -> None:
        try:
            app = adsk.core.Application.get()
            ui = app.userInterface

            # Get the command that was created.
            cmd = adsk.core.Command.cast(args.command)

            # Connect to the command destroyed event.
            onDestroy = CommandDestroyHandler(self.shared_vars)
            cmd.destroy.add(onDestroy)
            self.handlers.append(onDestroy)

            # Connect to the input changed event.
            onInputChanged = CommandInputChangedHandler(self.shared_vars)
            cmd.inputChanged.add(onInputChanged)
            self.handlers.append(onInputChanged)

            # Get the CommandInputs collection associated with the command.
            inputs = cmd.commandInputs

            # Set initial size of the dialog window
            dialog_width = 360
            cmd.setDialogInitialSize(dialog_width, 200)
            cmd.setDialogMinimumSize(dialog_width, 200)

            # Add the banner image at the top
            banner_image = inputs.addImageCommandInput("banner_image_id", "", config.banner_img_dir)
            banner_image.isFullWidth = True

            # Specify belt profile and thickness
            message = '<div align="center">Specify the Belt configuration</a></div>'
            inputs.addTextBoxCommandInput("profile_textBox", "", message, 1, True)
            inputs.addValueInput(
                "belt_thickness",
                "Belt thickness",
                "mm",
                adsk.core.ValueInput.createByString("5 mm"),
            )

            dropdown = inputs.addDropDownCommandInput(
                "belt_profile",
                "Belt profile",
                adsk.core.DropDownStyles.TextListDropDownStyle,
            )
            selected = True
            for profile in BeltProfile.all_profiles_dict:
                dropdown.listItems.add(profile, selected, "")
                selected = False

            # Select a construction plane (to be drawn on)
            planeSelection = inputs.addSelectionInput(
                "plane_selection",
                "Select Belt construction plane.",
                "Select a Construction plane. The timing belt will be symmetrically extruded from this plane.",
            )
            planeSelection.setSelectionLimits(0, 1)
            planeSelection.addSelectionFilter("ConstructionPlanes")

            # Topology table
            message = '<div align="center">Select pulley positions and specify the topology</a></div>'
            inputs.addTextBoxCommandInput("topology_textBox", "", message, 1, True)

            # Create table input
            masterSelection = inputs.addSelectionInput(
                "master_selection", "Select Pulley Position", "Select a sketch point or circular edge"
            )
            masterSelection.setSelectionLimits(0, 1)

            # Add filter
            filter_list = ["Vertices", "SketchPoints", "ConstructionPoints", "CircularEdges"]

            for filter in filter_list:
                masterSelection.addSelectionFilter(filter)

            tableInput = inputs.addTableCommandInput("table", "Table", 3, "1:1:1")
            addRowToTable(tableInput, self.shared_vars.rowNumber)
            self.shared_vars.rowNumber += 1

            # Add inputs into the table.
            addButtonInput = inputs.addBoolValueInput("tableAdd", "Add", False, "", True)
            tableInput.addToolbarCommandInput(addButtonInput)
            deleteButtonInput = inputs.addBoolValueInput("tableDelete", "Delete", False, "", True)
            tableInput.addToolbarCommandInput(deleteButtonInput)

            checkbox = inputs.addBoolValueInput("reverse", "Reverse/Mirror", True, "", False)
            checkbox.tooltip = "Reverses the order of the table, i.e mirrors the belt."

            message = '<div align="center">Make selection for the tensioner pulley</a></div>'
            inputs.addTextBoxCommandInput("tensioner_textBox", "", message, 1, True)

            # Tensioner pulley selection inputs
            tensionerSelection = inputs.addSelectionInput(
                "tensioner_selection", "Select a pull direction", "Select a cylindrical face or sketch point"
            )
            tensionerSelection.setSelectionLimits(0, 1)

            # Add filter
            filter_list = ["Vertices", "SketchPoints", "ConstructionPoints", "CircularEdges"]

            for filter in filter_list:
                tensionerSelection.addSelectionFilter(filter)

            spinner = inputs.addIntegerSpinnerCommandInput("tensioner_idx", "Tensioner row index", 0, 100, 1, 0)
            spinner.tooltip = "Select the index of the tensioner to use for the belt path."

            slider_tensioner = inputs.addFloatSliderCommandInput(
                "slider_tensioner", "Slide tensioner pulley", "", 0.0, 1.0, False
            )
            slider_tensioner.valueOne = 0.0
            slider_tensioner.tooltip = "The sliding motion may look choppy because only valid positions are previewed."

            onExecutePreview = CommandExecutePreviewHandler(self.shared_vars)
            onExecute = CommandExecuteHandler(self.shared_vars)

            cmd.executePreview.add(onExecutePreview)
            cmd.execute.add(onExecute)

            self.handlers.extend([onExecutePreview, onExecute])

        except:  # noqa: E722
            ui.messageBox(f"Failed:\n{traceback.format_exc()}")
