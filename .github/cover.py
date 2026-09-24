import matplotlib.pyplot as plt
import matplotlib.collections as mcoll
import os
import sys
import math


from belt_geometry import BeltSystem, SmoothPulley, ToothedPulley, GT2_2MM, BeltFace, Belt, Vec2
# Generates the cover image

profile = GT2_2MM

p1 = ToothedPulley(
    teeth=40,
    coords=(0, 15),
    profile=profile,
    name="Pulley 1",
)

p2 = SmoothPulley(
    diameter=10 * 2 / math.pi,
    coords=(35, 10),
    profile=profile,
    name="Idler",
)

p3 = ToothedPulley(
    teeth=25,
    coords=(70, 0),
    profile=profile,
    name="Pulley 2",
)

routing = [BeltFace.FRONT, BeltFace.BACK, BeltFace.FRONT]
midline = Belt(circles=[p1, p2, p3], topology=routing, allow_crossing=True)

belt_length = midline.total_length

target_length = belt_length // profile.pitch * profile.pitch
midline.find_movable_circle_position(target_length, 1, Vec2([0, -1]), method="secant")

system = BeltSystem(
    belt=midline,
    profile=profile,
)

fig, ax = system.plot(show=False, detailed=False)
fig.set_size_inches(12.8, 6.4)

# Multi-colored text separated into three aligned blocks
base_y = 22
base_x = 22

font_size = 66

ax.text(base_x, base_y, "Timing", fontsize=font_size, color="#E04006", fontweight="bold")
ax.text(base_x + 38, base_y, "Belt", fontsize=font_size, color="#3776AB", fontweight="bold")
ax.text(base_x + 3.5, base_y*0.6, "Generator", fontsize=font_size, color="#E9B815", fontweight="bold")


ax.set_xlim(-16, 80)
ax.set_ylim(-10, 30)
ax.axis("off")

# Author watermark
ax.text(-14, -8, "Made by: Vincent Wallsten", fontsize=15, color="black", ha="left")

ax.set_xlim(-16, 80)
ax.set_ylim(-10, 30)


ax.axis("off")

fig.savefig("cover.png", dpi=300, bbox_inches="tight", pad_inches=0.1)
plt.show()