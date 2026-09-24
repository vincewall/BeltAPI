import matplotlib.pyplot as plt
import matplotlib.collections as mcoll
import os
import sys
import math


from belt_geometry import BeltSystem, SmoothPulley, ToothedPulley, GT3_3MM, BeltFace, Belt, Vec2


# Generates the cover image

profile = GT3_3MM

p1 = ToothedPulley(
    teeth=18,
    coords=(-30, 0),
    profile=profile,
    name="Pulley 1",
)


p2 = ToothedPulley(
    teeth=18,
    coords=(30, 0),
    profile=profile,
    name="Pulley 2",
)

p3 = SmoothPulley(
    diameter=8,
    coords=(0, 0),
    profile=profile,
    name="Idler",
)
routing = [BeltFace.FRONT, BeltFace.FRONT, BeltFace.BACK]
midline = Belt(circles=[p1, p2, p3], topology=routing, allow_crossing=True)

belt_length = midline.total_length

target_length = belt_length // profile.pitch * profile.pitch
midline.find_movable_circle_position(target_length, 2, Vec2([0, 1]), method="secant")

system = BeltSystem(
    belt=midline,
    profile=profile,
)

fig, ax = system.plot(show=False, detailed=False)
ax.set_aspect("equal", adjustable="datalim")
ax.axis("off")
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

for collection in ax.collections:
    if isinstance(collection, mcoll.LineCollection):
        collection.set_linewidth(0.5)

aspect = 8/14
width_pixels = 360
width_inch = width_pixels / 300 # 1 dpi = 300 pixels as specified in the savefig
fig.set_size_inches(width_inch, width_inch*aspect)


# Multi-colored text separated into three aligned blocks
base_y = 25
base_x = -30

font_size = 9

ax.text(base_x, base_y, "Timing", fontsize=font_size, color="#E04006", fontweight="bold")
ax.text(base_x + 38, base_y, "Belt", fontsize=font_size, color="#3776AB", fontweight="bold")
ax.text(base_x + 3.5, base_y*0.6, "Generator", fontsize=font_size, color="#E9B815", fontweight="bold")


# Author watermark
ax.text(-11., -9, "By: Vincent", fontsize=4, color="black", ha="left")

ax.axis("off")
ax.set_ylim(-6, 30)


fig.savefig("dialog_banner.png", dpi=300)

plt.show()

print(width_inch)