"""
This file acts as the main launcher module for the Belt API script.
"""

import os
import sys
import traceback

import adsk.core  # type: ignore

# Workaround for importing submodules
dir = os.path.dirname(os.path.abspath(__file__))
if dir not in sys.path:
    sys.path.append(dir)


def clear_modules(PACKAGE_NAME):
    """
    Clears sys.modules so changes take affect immediately
    """

    modules_to_clear = [
        mod_name for mod_name in list(sys.modules.keys())
        if mod_name == PACKAGE_NAME or mod_name.startswith(f"{PACKAGE_NAME}.")
    ]

    for mod_name in modules_to_clear:
        del sys.modules[mod_name]

clear_modules("handlers")
clear_modules("belt_geometry")

from handlers.command_created_handler import CommandCreatedHandler
from handlers.shared_vars import SharedVars

_handlers = []

def run(context) -> None:
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        shared_vars = SharedVars()

        cmdDef = ui.commandDefinitions.itemById("BeltAPICommand")
        if not cmdDef:
            cmdDef = ui.commandDefinitions.addButtonDefinition(
                "BeltAPICommand", "Timing Belt API", "Generate a timing belt based on inputs"
            )

        onCommandCreated = CommandCreatedHandler(shared_vars)
        cmdDef.commandCreated.add(onCommandCreated)

        _handlers.append(onCommandCreated)

        cmdDef.execute()

        adsk.autoTerminate(False)
        
    except Exception:  # noqa: BLE001
        app = adsk.core.Application.get()
        if app.userInterface:
            app.userInterface.messageBox(f"Failed:\n{traceback.format_exc()}")