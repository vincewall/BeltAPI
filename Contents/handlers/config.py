import os

import adsk.core  # type: ignore

# Colors
green = adsk.core.Color.create(0, 255, 0, 255)
blue = adsk.core.Color.create(0, 150, 255, 255)
gray = adsk.core.Color.create(128, 128, 128, 255)
red = adsk.core.Color.create(255, 0, 0, 255)
yellow = adsk.core.Color.create(255, 255, 0, 255)
black = adsk.core.Color.create(0, 0, 0, 255)

# Min values
min_diameter = 5
min_tooth_count = 6

# Conversions
to_mm = 10

# Crosshair marker and banner images
script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(script_dir)

crosshair_img_dir = os.path.join(parent_dir, "resources", "crosshair.png")
banner_img_dir = os.path.join(parent_dir, "resources", "dialog_banner.png")
