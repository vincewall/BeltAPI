import traceback

import adsk.core  # type: ignore

from .shared_vars import SharedVars


class CommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self, shared_vars: SharedVars) -> None:
        super().__init__()
        self.shared_vars = shared_vars


    def notify(self, args: object) -> None:
        try:
            app = adsk.core.Application.get()
            ui = app.userInterface

            if self.shared_vars.preview_graphics:
                self.shared_vars.preview_graphics.deleteMe()
                self.shared_vars.preview_graphics = None

            adsk.terminate()
        except:  # noqa: E722
            ui.messageBox(f"Failed:\n{traceback.format_exc()}")