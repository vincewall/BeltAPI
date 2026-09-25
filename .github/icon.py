import matplotlib.pyplot as plt
import matplotlib.collections as mcoll
import os
import sys
import math


from belt_geometry import BeltSystem, SmoothPulley, ToothedPulley, GT3_3MM, BeltFace, Belt, Vec2


# Generates the icon

profile = GT3_3MM

disp = 8

tooth_count = 12

p1 = ToothedPulley(
    teeth=tooth_count,
    coords=(-disp, disp),
    profile=profile,
    name="Pulley 1",
)

p = 0.5
p2 = SmoothPulley(
    diameter=10,
    coords=(disp*p, disp*p),
    profile=profile,
    name="Idler",
)

p3 = ToothedPulley(
    teeth=tooth_count,
    coords=(disp, -disp),
    profile=profile,
    name="Pulley 3",
)

p4 = ToothedPulley(
    teeth=tooth_count,
    coords=(-disp, -disp),
    profile=profile,
    name="Pulley 2",
)

routing = [BeltFace.FRONT, BeltFace.BACK, BeltFace.FRONT, BeltFace.FRONT]
midline = Belt(circles=[p1, p2, p3, p4], topology=routing, allow_crossing=True)

belt_length = midline.total_length

target_length = belt_length // profile.pitch * profile.pitch
midline.find_movable_circle_position(target_length, 1, Vec2([1, 1]), method="secant")

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

aspect = 1
width_pixels = 120
width_inch = width_pixels / 300 # 1 dpi = 300 pixels as specified in the savefig
fig.set_size_inches(width_inch, width_inch*aspect)


# Multi-colored text separated into three aligned blocks
base_y = 25
base_x = -30

font_size = 9

#ax.text(base_x, base_y, "Timing", fontsize=font_size, color="#E04006", fontweight="bold")
#ax.text(base_x + 38, base_y, "Belt", fontsize=font_size, color="#3776AB", fontweight="bold")
#ax.text(base_x + 3.5, base_y*0.6, "Generator", fontsize=font_size, color="#E9B815", fontweight="bold")

ax.axis("off")



fig.savefig("icon.png", dpi=300)

plt.show()

print(width_inch)