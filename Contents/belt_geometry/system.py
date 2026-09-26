"""
Profiled Belt generation script.

Reduced version of beltpy.
"""


from __future__ import annotations

import copy
import math

from .profiles import BeltProfile, Pt
from .replacements import Vec2
from .solver import Belt, BeltFace, Circle
from .tooth import ProfileTransform


class BeltSystemError(Exception):
    """Base class for all errors raised by the Belt system generator."""


class BeltLengthError(BeltSystemError):
    """Raised when the length is not evenly divisible by the pitch."""


class ToothedPulley(Circle):
    """A synchronous pulley driven by tooth count and pitch."""

    def __init__(self, coords: Vec2 | tuple[float, float] | list[float], teeth: int, profile: BeltProfile, name: str | None = None) -> None:
        self.teeth = teeth
        self.system_profile = profile
        coords = Vec2(coords)
        r = (self.teeth * self.system_profile.pitch) / (2 * math.pi)
        super().__init__(r=r, coords=(coords.x, coords.y), name=name)


class SmoothPulley(Circle):
    """A smooth tensioning pulley driven with an arbitrary diameter."""

    def __init__(
        self, coords: Vec2 | tuple[float, float] | list[float], diameter: float, profile: BeltProfile, name: str | None = None) -> None:
        self.diameter = diameter

        coords = Vec2(coords)
        offset = abs(profile.base_points_left[Pt.BACK_CENTER].y)
        radius = diameter / 2.0 + offset

        super().__init__(r=radius, coords=(coords.x, coords.y), name=name)


class BeltSystem:
    """
    Creates a set of individual teeth with a specified profile that together constitute
    a belt system.
    """

    def __init__(
        self,
        belt: Belt,
        profile: BeltProfile,
    ) -> None:
        """
        Creates the belt system from a solved midline belt and a tooth profile.

        The belt must already have a length that is evenly divisible by the profile
        pitch. Use the optimization routines on the `pybeltsolver.Belt` instance
        before passing it here if its length needs to be adjusted.

        Args:
            belt (Belt): A midline belt path from `pybeltsolver`. Make sure the length
                is evenly divisible by the profile pitch!
            profile (BeltProfile): The Belt profile to be used. Specifically, the pitch
                is needed to generate all teeth on the belt path.
        """

        self.profile = profile

        self.tooth_coords = []
        self.tooth_normals = []
        self.tooth_tangents = []

        # Transition teeth
        self.transition_idx = []
        self.transition_tangents = {}
        self.transition_normals = {}
        self.transition_diameters = {}
        self.transition_partition_angles = {}

        self.blocks = []

        # For plotting purposes
        self.transition_tangent_lines = {}
        self.transition_normal_lines = {}
        self.tooth_tangent_lines = []
        self.tooth_normal_lines = []

        self.num_teeth = 0

        # The midline path should be provided as input
        self.midline_path = belt

        # Validate the length
        self._validate_length()

        # The backing path
        offset = abs(profile.base_points_left[Pt.BACK_CENTER].y)
        circles_back = []

        for _topology, circle in zip(belt.topology, belt.circles):
            circle_back = copy.copy(circle)

            # Add or subtract the offset depending on the topology
            if _topology == BeltFace.FRONT:
                circle_back.r = circle_back.r + offset
            else:
                circle_back.r = circle_back.r - offset
            if circle_back.name is not None:
                circle_back.name = circle_back.name + "_backing"
            circles_back.append(circle_back)

        self.backing_path = Belt(
            circles=circles_back,
            topology=belt.topology,
            do_validate=belt.do_validate,
            allow_crossing=belt.allow_crossing,
        )

        self._walk_belt()

        self._create_teeth()

    def plot(self, show: bool = True, detailed: bool = False) -> tuple:
        """
        Plot the belt system.

        Args:
            show (bool, optional): Show the plot after creation or not. Defaults to True.
            detailed (bool, optional): Shows a more detailed view on how the belt is
                constructed. Defaults to False.

        Returns:
            tuple[Figure, Axes]: Matplotlib Figure and Axes object.
        """
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection

        fig, ax = plt.subplots()

        # Midline
        segs = self.midline_path.lines + self.midline_path.arcs
        line_segs = LineCollection(
            [[(point.x, point.y) for point in seg] for seg in segs],
            linestyle="--",
            color="gray",
            label="Midline",
        )
        ax.add_collection(line_segs)

        # Backing
        segs = self.backing_path.lines + self.backing_path.arcs
        line_segs = LineCollection(
            [[(point.x, point.y) for point in seg] for seg in segs],
            linestyle="-",
            color="black",
            label="Backing",
        )
        ax.add_collection(line_segs)

        # Mainly used for debug
        if detailed:
            # Plot arc endpoints
            circles = self.midline_path.circles
            entry_points = [
                self.midline_path.wrap_points[circle]["entry"] for circle in circles
            ]
            exit_points = [
                self.midline_path.wrap_points[circle]["exit"] for circle in circles
            ]
            x = [point.x for point in entry_points]
            y = [point.y for point in entry_points]
            ax.scatter(x, y, marker="s", label="Entry arc endpoints")

            x = [point.x for point in exit_points]
            y = [point.y for point in exit_points]
            ax.scatter(x, y, marker="v", label="Exit arc endpoints")

            # Plot tooth coords on midline
            x = [point.x for point in self.tooth_coords]
            y = [point.y for point in self.tooth_coords]
            ax.scatter(x, y, marker="o", color="r", label="Tooth Coords")

            # Plot normal arrows
            self._plot_arrows(ax, plot_norms_tangs=True, plot_trans_norms_tangs=True)

            # Plot tooth numbers
            self._plot_tooth_numbers(ax)

            # Plot the backing transitional points
            self._plot_backing_partition_points(ax)

        self._plot_teeth(ax)

        ax.autoscale()
        ax.set_aspect("equal")

        plt.show() if show else None

        return fig, ax

    def _plot_backing_partition_points(self, ax) -> None:
        """Plots the backing transitional points"""

        path = self.backing_path
        circles = path.circles

        p = []
        for circle in circles:
            entry_line = path.wrap_lines[circle]["entry"]
            exit_line = path.wrap_lines[circle]["exit"]

            p1, _ = entry_line  # p1 at current circle exit, p2 at next circle entry
            _, p2 = exit_line  # p1 at current circle exit, p2 at next circle entry
            p.append(p1)
            p.append(p2)

        px = [point.x for point in p]
        py = [point.y for point in p]

        ax.scatter(px, py, c="orange", marker="o", label="Backing Partition")

    def _plot_teeth(self, ax, plot_profile_points: bool = False) -> None:
        """
        Plots all the teeth in the belt system.

        Args:
            ax (Axes): Axes object.
            plot_profile_points (bool, optional): Plots all the individual points making
            up the tooth profile it True. Defaults to False.
        """
        from matplotlib.collections import LineCollection

        all_tooth_segs = []

        p_p = []
        for tooth in self.teeth:
            # Grab all the arcs and lines that make up the tooth outline
            for segment in tooth._create_all_arcs(side="left", only_top=True):
                all_tooth_segs.append([(point.x, point.y) for point in segment])
            for segment in tooth._create_all_arcs(side="right", only_top=True):
                all_tooth_segs.append([(point.x, point.y) for point in segment])

            for segment in tooth._create_all_straight_line_segments(side="left", only_top=False):
                all_tooth_segs.append([(point.x, point.y) for point in segment])
            for segment in tooth._create_all_straight_line_segments(side="right", only_top=False):
                all_tooth_segs.append([(point.x, point.y) for point in segment])

            p_p.append(tooth.global_points)

            if hasattr(tooth, "pl_trans"):
                p_p.append(tooth._transform(tooth.pl_trans))

        if plot_profile_points:
            profile_points = [point for group in p_p for point in group]
            px = [point.x for point in profile_points]
            py = [point.y for point in profile_points]
            ax.scatter(px, py, c="red", marker="x", label="Tooth Profile")

        # Add them all to the main plot as solid black lines
        tooth_collection = LineCollection(
            all_tooth_segs,
            linestyle="solid",
            color="black",
            linewidth=1.5,
            label="Teeth",
        )

        ax.add_collection(tooth_collection)

    def _plot_arrows(
        self,
        ax,
        plot_norms_tangs: bool = True,
        plot_trans_norms_tangs: bool = True,
    ) -> None:
        """
        Plots teeth normal and tangent arrows.

        Args:
            ax (Axes): Axes object.
            plot_norms_tangs (bool, optional): Plots normal arrows in the tooth center.
                Defaults to true
            plot_trans_norms_tangs (bool, optional): Plots tangent arrows in the tooth
            center. Defaults to True.
        """

        scale_factor = self.profile.pitch / 2

        if plot_norms_tangs:
            for tooth_normal_line in self.tooth_normal_lines:
                p1, p2 = tooth_normal_line
                p1_vec = p1
                p2_vec = p2

                dx_t = (p2_vec.x - p1_vec.x) * scale_factor
                dy_t = (p2_vec.y - p1_vec.y) * scale_factor

                # Tangent arrow
                x0, y0 = p1_vec.x, p1_vec.y
                ax.arrow(x0, y0, dx_t, dy_t, width=0.015, fc="red", ec="red")

            # Tangent
            for tooth_tangent_line in self.tooth_tangent_lines:
                p1, p2 = tooth_tangent_line
                p1_vec = p1
                p2_vec = p2

                dx_t = (p2_vec.x - p1_vec.x) * scale_factor
                dy_t = (p2_vec.y - p1_vec.y) * scale_factor

                # Tangent arrow
                x0, y0 = p1_vec.x, p1_vec.y
                ax.arrow(x0, y0, dx_t, dy_t, width=0.015, fc="blue", ec="blue")

        if plot_trans_norms_tangs:
            for tangent, normal in zip(
                self.transition_tangent_lines.values(),
                self.transition_normal_lines.values(),
            ):
                p1, p2 = tangent
                p1_vec = p1
                p2_vec = p2

                dx_t = (p2_vec.x - p1_vec.x) * scale_factor
                dy_t = (p2_vec.y - p1_vec.y) * scale_factor

                # Tangent arrow
                x0, y0 = p1_vec.x, p1_vec.y
                ax.arrow(x0, y0, dx_t, dy_t, width=0.015, fc="green", ec="green")

                p1, p2 = normal
                p1_vec = p1
                p2_vec = p2

                dx_t = (p2_vec.x - p1_vec.x) * scale_factor
                dy_t = (p2_vec.y - p1_vec.y) * scale_factor

                # Normal arrow
                x0, y0 = p1_vec.x, p1_vec.y
                ax.arrow(x0, y0, dx_t, dy_t, width=0.015, fc="green", ec="green")

    def _plot_tooth_numbers(self, ax) -> None:
        """
        Plots the tooth numbering offset from the teeth. Transitional teeth are
        highlighted in green.
        """
        from matplotlib import patches
        from matplotlib.textpath import TextPath
        from matplotlib.transforms import Affine2D

        for i, (coord, normal) in enumerate(
            zip(self.tooth_coords, self.tooth_normal_lines)
        ):
            x, y = coord.x, coord.y
            p1, p2 = normal
            n = p2 - p1

            offset = -1
            dx, dy = n.x * offset, n.y * offset

            text_polygon = TextPath((0, 0), str(i + 1), size=0.8)

            # Adjust for the size of the text bounding box
            bbox = text_polygon.get_extents()

            center_x = (bbox.x0 + bbox.x1) / 2.0
            center_y = (bbox.y0 + bbox.y1) / 2.0

            position_transform = (
                Affine2D().translate(x + dx - center_x, y + dy - center_y)
                + ax.transData
            )

            color = "green" if i in self.transition_idx else "blue"
            patch = patches.PathPatch(
                text_polygon,
                transform=position_transform,
                facecolor=color,
                edgecolor="none",
                zorder=10,
            )
            ax.add_patch(patch)

    def _walk_circle_section(
        self,
        offset: float,
        phi_1: float,
        circle: Circle,
        direction_sign: int,
        arc_length: float,
        tooth_idx: int,
    ) -> tuple[float, int]:
        """
        Helper function to walk a circle section and add teeth along the way.
        """

        pitch = self.profile.pitch
        theta_offset = -offset / circle.r * direction_sign
        d_theta = -pitch / circle.r * direction_sign
        current_dist = offset
        n = 0

        while arc_length > current_dist:
            theta_new = (phi_1 + theta_offset) + n * d_theta
            s = math.sin(theta_new)
            c = math.cos(theta_new)
            tooth_coord = Vec2(circle.x + c * circle.r, circle.y + s * circle.r)
            self.tooth_coords.append(tooth_coord)

            v_n = (Vec2(circle.x, circle.y) - tooth_coord) / circle.r * direction_sign
            self.tooth_normal_lines.append([tooth_coord, tooth_coord + v_n])
            self.tooth_normals.append(v_n)

            t_n = Vec2(-math.sin(theta_new), math.cos(theta_new)) * direction_sign
            self.tooth_tangent_lines.append([tooth_coord, tooth_coord + t_n])
            self.tooth_tangents.append(t_n)

            current_dist += pitch
            n += 1
            tooth_idx += 1

        return current_dist - arc_length, tooth_idx

    def _save_transition_data(
        self,
        offset: float,
        circle: Circle,
        direction_sign: int,
        previous_idx: int,
        tooth_idx: int,
        block_diameter: float | None,
        wrap_index: bool = False,
    ) -> int:
        """
        Helper function to save the transition data for a given tooth.
        """

        pitch = self.profile.pitch
        transition_idx = tooth_idx - 1 if offset > pitch / 2 else tooth_idx
        if wrap_index:
            transition_idx = int(transition_idx % self.num_teeth)
            start = (previous_idx + 1) % self.num_teeth
            end = (transition_idx - 1) % self.num_teeth
        else:
            start = previous_idx + 1
            end = transition_idx - 1

        self.transition_idx.append(transition_idx)
        self.blocks.append([start, end, block_diameter])

        s_dist = -(pitch - offset) if offset > pitch / 2 else offset
        self.transition_partition_angles[transition_idx] = (
            s_dist / circle.r
        ) * direction_sign
        self.transition_diameters[transition_idx] = circle.r * 2 * direction_sign
        return transition_idx

    def _walk_straight_segment(
        self,
        offset: float,
        circle: Circle,
        direction_sign: int,
        p1: Vec2 | tuple[float, float] | list[float],
        p2: Vec2 | tuple[float, float] | list[float],
        phi_2: float,
        tooth_idx: int,
    ) -> tuple[float, int, Vec2, Vec2]:
        """
        Helper function to walk a straight segment and add teeth along the way.
        """
        p1_vec = p1
        p2_vec = p2
        center = Vec2(circle.x, circle.y)
        v_n = (center - p1_vec) / (center - p1_vec).norm() * direction_sign
        t_n = Vec2(-math.sin(phi_2), math.cos(phi_2)) * direction_sign
        vs = p2_vec - p1_vec
        length = vs.norm()
        vs_n = vs / length
        current_dist = offset

        while current_dist < length:
            tooth_coord = p1_vec + vs_n * current_dist
            self.tooth_coords.append(tooth_coord)
            self.tooth_normal_lines.append([tooth_coord, tooth_coord + v_n])
            self.tooth_tangent_lines.append([tooth_coord, tooth_coord + t_n])
            self.tooth_normals.append(v_n)
            self.tooth_tangents.append(t_n)
            current_dist += self.profile.pitch
            tooth_idx += 1

        return current_dist - length, tooth_idx, v_n, t_n

    def _save_transition_lines(
        self,
        first_idx: int,
        second_idx: int,
        p1: Vec2 | tuple[float, float] | list[float],
        p2: Vec2 | tuple[float, float] | list[float],
        v_n: Vec2 | tuple[float, float] | list[float],
        t_n: Vec2 | tuple[float, float] | list[float],
    ) -> None:
        """
        Helper function to save the transition lines for plotting and debugging.
        """
        self.transition_normal_lines[first_idx] = [p1, p1 + v_n]
        self.transition_tangent_lines[first_idx] = [p1, p1 - t_n]
        self.transition_normals[first_idx] = v_n
        self.transition_tangents[first_idx] = -t_n

        self.transition_normal_lines[second_idx] = [p2, p2 + v_n]
        self.transition_tangent_lines[second_idx] = [p2, p2 + t_n]
        self.transition_normals[second_idx] = v_n
        self.transition_tangents[second_idx] = t_n

    def _walk_belt(self) -> None:
        """
        Walks the midline path of the belt and places teeth along the way.
        """
        path = self.midline_path
        circles = path.circles
        offset = self.profile.pitch / 2
        tooth_idx = 0
        previous_transition_idx = 0

        for idx, circle in enumerate(circles):
            direction_sign = 1 if path.topology[idx] == BeltFace.FRONT else -1

            # Walk the circle section and add teeth along the way
            offset, tooth_idx = self._walk_circle_section(
                offset,
                path.wrap_angles[circle]["entry"],
                circle,
                direction_sign,
                path.arcs_lengths[idx],
                tooth_idx,
            )

            # Extract transition idx and save data
            circle_transition_idx = self._save_transition_data(
                offset,
                circle,
                direction_sign,
                previous_transition_idx,
                tooth_idx,
                circle.r * 2 * direction_sign,
            )

            # Walk the straight section and add teeth along the way
            p1, p2 = path.wrap_lines[circle]["exit"]
            offset, tooth_idx, v_n, t_n = self._walk_straight_segment(
                offset,
                circle,
                direction_sign,
                p1,
                p2,
                path.wrap_angles[circle]["exit"],
                tooth_idx,
            )

            # Save for next iteration
            next_idx = (idx + 1) % len(circles)
            next_circle = circles[next_idx]
            next_direction_sign = (
                1 if path.topology[next_idx] == BeltFace.FRONT else -1
            )

            # Extract transition idx and save data
            line_transition_idx = self._save_transition_data(
                offset,
                next_circle,
                next_direction_sign,
                circle_transition_idx,
                tooth_idx,
                None,
                wrap_index=True,
            )

            # Save the transition lines for plotting and debugging
            self._save_transition_lines(
                circle_transition_idx, line_transition_idx, p1, p2, v_n, t_n
            )
            previous_transition_idx = line_transition_idx

        # Make sure we have no wrap-around issues (might be redundant)
        start = (previous_transition_idx + 1) % self.num_teeth
        self.blocks[0][0] = start
        self.transition_idx = sorted(set(self.transition_idx))

    def _create_teeth(self) -> None:
        """
        Creates all the tooth objects along the midline path.
        """
        teeth_by_index = [None] * self.num_teeth

        for idx in self.transition_idx:
            position = self.tooth_coords[idx]
            nx, ny = self.tooth_normals[idx].x, self.tooth_normals[idx].y
            phi_tooth = math.atan2(ny, nx) - (math.pi / 2)

            trans_t = self.transition_tangents[idx]
            trans_n = self.transition_normals[idx]
            partition_angle = self.transition_partition_angles[idx]
            partition_angle = (partition_angle + math.pi) % (2 * math.pi) - math.pi

            tooth = ProfileTransform(
                profile=self.profile,
                position=position,
                angle=phi_tooth,
                D=self.transition_diameters[idx],
                global_transition_tangent=trans_t,
                transform_tangent=True,
                partition_angle=partition_angle,
                global_partition_normal=trans_n,
            )

            teeth_by_index[idx] = tooth

        for start, end, D in self.blocks:
            tooth_base = ProfileTransform(
                profile=self.profile,
                position=[0, 0],
                angle=0,
                D=D,
            )

            for idx in range(start, end + 1):
                tooth = copy.deepcopy(tooth_base)

                position = self.tooth_coords[idx]

                nx, ny = self.tooth_normals[idx].x, self.tooth_normals[idx].y
                phi_tooth = math.atan2(ny, nx) - (math.pi / 2)

                tooth.position = position
                tooth.angle = phi_tooth
                tooth._rigid_body_transform()

                teeth_by_index[idx] = tooth

        missing_indices = [
            idx for idx, tooth in enumerate(teeth_by_index) if tooth is None
        ]
        if missing_indices:
            raise BeltSystemError(
                f"Failed to create teeth at indices: {missing_indices}"
            )

        self.teeth = teeth_by_index
    
    def _validate_length(self):
        """
        Validates that the midline path length is evenly divisible by the profile pitch.
        """
        tol = 1e-6

        path = self.midline_path
        length = path.total_length
        pitch = self.profile.pitch
        num_teeth = round(length / pitch)

        if abs(length - num_teeth * pitch) > tol:
            raise BeltLengthError(
                f"The midline belt length ({length:.6f}) is not evenly divisible "
                f"by the pitch ({pitch:.6f}). Adjust the Belt with pybeltsolver "
                "before constructing a BeltSystem."
            )

        self.num_teeth = num_teeth