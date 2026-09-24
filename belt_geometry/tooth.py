"""
Tooth generation script.

Reduced version of pytooth from beltpy.
"""

import math

from .profiles import Ctr, Pt
from .replacements import Vec2


class ProfileTransform:
    """
    Takes a base tooth profile and applies transformations in the form of rotations,
    translations and distortion.
    """

    def __init__(
        self,
        profile,
        position,
        angle,
        D=None,
        global_transition_tangent=None,
        transform_tangent=True,
        partition_angle=None,
        global_partition_normal=None,
    ):
        """
        Initializes and applies transformations in the form of rotations, translations
        and distortion for any tooth profile. For pure bending, the diameter `D` only
        has to be passed. For a transitioning tooth, `D` along with
        `global_transition_tangent` and the `partition_angle` also have to be passed.

        Currently, the class does not verify if the user input is valid or not. Be aware
        that physically impossible configurations will still produce results.

        Args:
            position (list): Should contain [x, y] coordinates of tooths origin
                on pitch line.
            angle (float): Rotation of tooth in radians.
            D (float, optional): Curve diameter (distortion). Positive values
                create a convex distortion (wrap around pulley), negative values
                create a concave distortion (pushed by idle pulley). Defaults to
                None, which means no distortion.
            global_transition_tangent: The globally defined tangent vector
                at the point on the midline that marks the transition between the curved
                and straight segment of the tooth/midline. Must point in the direction
                of the straight segment. Defaults to None, which mean no distortion.
            transform_tangent (bool): For debug or testing purposes. False sets
                the local transition tangent equal to the global transition tangent.
                Defaults to True.
            partition_angle (float): Angle (in radians) between the partition point on
                the midline and the line running through the midline center (0,0). May
                be positive or negative. Defaults to None.
        """
        # Transition check
        self.profile = profile
        self.is_transitioning = global_transition_tangent is not None and partition_angle is not None

        # Transition tangent needs to be specified in the local frame
        if self.is_transitioning and transform_tangent:
            c = math.cos(angle)
            s = math.sin(angle)
            self.transition_tangent = Vec2(
                global_transition_tangent.x * c + global_transition_tangent.y * s,
                -global_transition_tangent.x * s + global_transition_tangent.y * c,
            )
        else:
            self.transition_tangent = global_transition_tangent

        # Transition tangent needs to be specified in the local frame
        if self.is_transitioning and transform_tangent:
            self.partition_normal = Vec2(
                global_partition_normal.x * c + global_partition_normal.y * s,
                -global_partition_normal.x * s + global_partition_normal.y * c,
            )
        else:
            self.partition_normal = global_partition_normal

        self.partition_angle = partition_angle
        # Initial values for tangent pointing side
        self.side = None
        self.opposite_side = None
        # Position and curvature
        self.position = Vec2(position)
        self.angle = angle
        self.D = D

        # Base config (straight tooth)
        self._initialize_arrays()
        self.tooth_thickness = (
            self.local_points_left[Pt.FLAT_UPPER_CORNER] - self.local_points_left[Pt.BACK_LOWER_CORNER]
        ).norm()

        # Curve and transition logic
        if D:
            self.curvature_c = Vec2(0, D / 2)
            self.global_curve_center = self._transform(self.curvature_c)

            if self.is_transitioning:
                self._compute_invariants()
                self._compute_transition_tooth()
            else:
                sign_D = 1 if D > 0 else -1 if D < 0 else 0
                R_abs = abs(D / 2)
                self.Op = Vec2(0, sign_D * R_abs)
                self.straight_seg_length = 0
                self._compute_curved_tooth()

        # Rigid body transform and concatenate all points and centers
        self._rigid_body_transform()
        self.global_points = self.global_points_left + self.global_points_right
        self.global_centers = self.global_centers_left + self.global_centers_right

    def _rigid_body_transform(self):
        """Performs the rigid body transform on all local arrays."""
        self.global_points_left = [self._transform(p) for p in self.local_points_left]
        self.global_centers_left = [self._transform(p) for p in self.local_centers_left]
        self.global_points_right = [self._transform(p) for p in self.local_points_right]
        self.global_centers_right = [self._transform(p) for p in self.local_centers_right]

        if getattr(self, "curvature_c", None) is not None:
            self.global_curve_center = self._transform(self.curvature_c)

    def _mirror_arrays(self):
        """Mirrors the left array on the right side."""
        self.local_points_right = [Vec2(-p.x, p.y) for p in self.local_points_left]
        self.local_centers_right = [Vec2(-p.x, p.y) for p in self.local_centers_left]

    def _initialize_arrays(self):
        """Copy left point arrays and mirror to the right side."""
        self.local_points_left = [
            Vec2(point.x, point.y) for point in self.profile.base_points_left
        ]
        self.local_centers_left = [
            Vec2(point.x, point.y) for point in self.profile.base_centers_left
        ]
        self._mirror_arrays()

    def _get_side_arrays(self, side):
        """Returns the points and centers for the requested side."""
        if side == "left":
            return self.local_points_left, self.local_centers_left
        return self.local_points_right, self.local_centers_right

    def _compute_curved_tooth(self):
        """Computes a tooth that is purely curved."""
        self._compute_root_fillet_arc()
        self._compute_new_tooth_side_geom_arc()
        # Just mirror the result as in the init
        self._mirror_arrays()

    def _compute_transition_tooth(self):
        """Computes a tooth that has a curved and straight segment."""
        side = self.side
        o_side = self.opposite_side
        self._compute_midline_segments(side, self.partition_angle)
        self._compute_tooth_center_rotation()
        self._compute_new_tooth_side_geom_straight(side)
        self._compute_root_fillet_straight(side)
        self._compute_new_tooth_side_geom_arc(o_side)
        self._compute_root_fillet_arc(o_side)
        self._compute_backing_transition()

    def _compute_tooth_center_rotation(self):
        """
        Bends the tip of the tooth based on the transition ratio. Longer straight
        segment -> tooth tip gets closer to being perpendicular to the transition
        tangent.
        """

        bend_angle = self.straight_seg_length / self.profile.pitch * self.partition_angle
        points_start = Pt.FLANK_FILL_START
        centers_start = Ctr.TIP
        c = math.cos(bend_angle)
        s = math.sin(bend_angle)
        self.local_points_left[points_start:] = [
            Vec2(p.x * c - p.y * s, p.x * s + p.y * c)
            for p in self.local_points_left[points_start:]
        ]
        self.local_points_right[points_start:] = [
            Vec2(p.x * c - p.y * s, p.x * s + p.y * c)
            for p in self.local_points_right[points_start:]
        ]
        self.local_centers_left[centers_start:] = [
            Vec2(p.x * c - p.y * s, p.x * s + p.y * c)
            for p in self.local_centers_left[centers_start:]
        ]
        self.local_centers_right[centers_start:] = [
            Vec2(p.x * c - p.y * s, p.x * s + p.y * c)
            for p in self.local_centers_right[centers_start:]
        ]

    def _compute_invariants(self):
        """
        Computes variables that will be used across multiple other functions.
        """
        if self.D is None:
            return

        self.side = "right" if self.transition_tangent.x > 0 else "left"
        self.opposite_side = "right" if self.side == "left" else "left"
        R_abs = abs(self.D / 2)
        sign_D = 1 if self.D > 0 else -1 if self.D < 0 else 0
        self.l_top = self.tooth_thickness - self.profile.pld
        _, self.straight_seg_length = self._compute_midline_segments(self.side, self.partition_angle)

        # Check if origin (0,0) is physically on the straight segment
        if self.straight_seg_length > self.profile.pitch / 2:
            L = self.straight_seg_length - self.profile.pitch / 2
        else:
            L = 0

        # Handle two separate cases
        if L > 0:
            # Origin is on straight segment
            s = math.sin(self.partition_angle * sign_D)
            c = math.cos(self.partition_angle * sign_D)
            dR = math.sqrt(R_abs**2 + L**2) - R_abs
            self.pl_trans = Vec2(R_abs * s, sign_D * R_abs * (1 - c) + dR)
            self.n_h_trans = Vec2(-sign_D * s, c)
            self.Op = Vec2(0, sign_D * R_abs + dR)
            
            # Test
            if self.partition_normal is not None:
                self.n_h_trans = self.partition_normal
            self.pl_trans = -self.transition_tangent * L
            self.Op = self.pl_trans + R_abs * self.n_h_trans * sign_D
        else:
            # Origin is on the curved segment
            s = math.sin(self.partition_angle * sign_D)
            c = math.cos(self.partition_angle * sign_D)
            self.pl_trans = Vec2(R_abs * s, sign_D * R_abs * (1 - c))
            self.n_h_trans = Vec2(-sign_D * s, c)
            self.Op = Vec2(0, sign_D * R_abs)

        self.curvature_c = self.Op
        self.global_curve_center = self._transform(self.curvature_c)

    def _compute_midline_segments(self, side, phi):
        """
        Computes the lengths of the straight and curved segments projecting
        from the transition boundary out to the outer edges of the tooth.

        Args:
            side (str): Straight segment side.
            phi (float): Partition angle.
        Returns:
            tuple: Contains (curved_length, straight_length).
        """

        half_pitch = self.profile.pitch / 2
        R_abs = abs(self.D / 2)
        sign_D = 1 if self.D > 0 else -1 if self.D < 0 else 0
        x_p = R_abs * phi * sign_D
        if side == "right":
            curved_length, straight_length = half_pitch + x_p, half_pitch - x_p
        else:
            curved_length, straight_length = half_pitch - x_p, half_pitch + x_p
        return (
            max(0.0, min(curved_length, self.profile.pitch)),
            max(0.0, min(straight_length, self.profile.pitch)),
        )

    def _transform(self, local_point):
        """Apply the standard 2D rotation matrix and translation."""
        # Standard 2D Rotation Matrix
        c = math.cos(self.angle)
        s = math.sin(self.angle)
        return Vec2(
            local_point.x * c - local_point.y * s + self.position.x,
            local_point.x * s + local_point.y * c + self.position.y,
        )

    def _compute_backing_transition(self):
        """Compute the backing transition points and assign to both arrays."""
        point = self.pl_trans - self.n_h_trans * self.l_top
        self.local_points_left[Pt.BACK_CENTER] = point
        self.local_points_right[Pt.BACK_CENTER] = point

    def _compute_new_tooth_side_geom_arc(self, side="left"):
        """
        Computes the new position of BACK_LOWER_CORNER and FLAT_UPPER_CORNER based on
        the curve diameter D. Achieved by calculating point pl (point on pitch line) and
        n_h (normal pointing from pitch line to curve center), then applying offsets.

        Args:
            side (str): Straight segment side.
        """

        points, _ = self._get_side_arrays(side)
        # The angle phi is derived assuming pitch line length is constant
        phi = 2 * self.profile.pitch / self.D
        angle = -phi / 2 if side == "left" else phi / 2
        pl = Vec2(math.sin(angle), -math.cos(angle)) * (self.D / 2) + self.Op

        if self.is_transitioning:
            Lc = self.profile.pitch - self.straight_seg_length  # Curved length
            R_abs = abs(self.D / 2)
            theta = Lc / R_abs
            V = self.pl_trans - self.Op
            T_curve = -self.transition_tangent
            cross_prod = V.x * T_curve.y - V.y * T_curve.x
            rot_dir = 1 if cross_prod > 0 else -1 if cross_prod < 0 else 0
            theta *= rot_dir
            c = math.cos(theta)
            s = math.sin(theta)
            pl = self.Op + Vec2(V.x * c - V.y * s, V.x * s + V.y * c)

        n_h = (self.Op - pl) * (2 / self.D)
        # Apply offset along n_h to find new points
        l_top = self.tooth_thickness - self.profile.pld
        points[Pt.BACK_LOWER_CORNER] = pl - n_h * l_top
        points[Pt.FLAT_UPPER_CORNER] = pl + n_h * self.profile.pld

    def _compute_new_tooth_side_geom_straight(self, side):
        """
        Compute the new BACK_LOWER_CORNER and FLAT_UPPER_CORNER coordinates for a
        straight segment.

        Args:
            side (str): Straight segment side.
        """

        points, _ = self._get_side_arrays(side)
        base_point = self.pl_trans + self.transition_tangent * self.straight_seg_length
        points[Pt.BACK_LOWER_CORNER] = base_point - self.n_h_trans * self.l_top
        points[Pt.FLAT_UPPER_CORNER] = base_point + self.n_h_trans * self.profile.pld

    def _compute_root_fillet_arc(self, side="left"):
        """
        Computes new arc center and intersect point for the root fillet based
        on the curve diameter D.

        Method used is based on the intersection of two circles with centers at
        the flank center (unchanged) and at the curve center with radii r1 = r_p
        + r_f and r2 = (D/2 - PLD) - r_f, where r_f is the original root fillet
        radius which is assumed to be unchanged. See link for mathematical
        details.

        Args:
            side (str): Straight segment side.

        References:
            Bourke, P. "Intersection of two circles."
            http://paulbourke.net/geometry/circlesphere/
        """

        points, centers = self._get_side_arrays(side)
        r1 = self.profile.r_flank + self.profile.r_root
        r2 = self.D / 2 - self.profile.pld - self.profile.r_root
        P_p = centers[Ctr.FLANK]
        x1, y1 = P_p.x, P_p.y
        x2, y2 = self.curvature_c.x, self.curvature_c.y
        R_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
        t1_x, t1_y = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
        t2_factor = (r1**2 - r2**2) / (2 * R_sq)
        t2_x, t2_y = t2_factor * (x2 - x1), t2_factor * (y2 - y1)
        sqrt_inner = 2 * (r1**2 + r2**2) / R_sq - (r1**2 - r2**2) ** 2 / R_sq**2 - 1
        t3_factor = 0.5 * math.sqrt(max(sqrt_inner, 0))
        t3_x, t3_y = t3_factor * (y2 - y1), t3_factor * (x1 - x2)

        # Calculate the two possible intersection points
        p_r1 = Vec2(t1_x + t2_x + t3_x, t1_y + t2_y + t3_y)
        p_r2 = Vec2(t1_x + t2_x - t3_x, t1_y + t2_y - t3_y)

        # Choose the intersect to the left or right depending on side
        if side == "left":
            p_r = p_r1 if p_r1.x < p_r2.x else p_r2
        elif side == "right":
            p_r = p_r1 if p_r1.x > p_r2.x else p_r2
        else:
            raise ValueError(f"Input '{side}' is not a valid side.")
        # Compute intersect points for tangency
        points[Pt.ROOT_FILL_START] = (
            self.curvature_c + (p_r - self.curvature_c) * ((self.D / 2) - self.profile.pld) / r2
        )
        points[Pt.FLANK_FILL_START] = P_p + (p_r - P_p) * (self.profile.r_flank / r1)
        centers[Ctr.ROOT] = p_r

    def _compute_root_fillet_straight(self, side="left"):
        """
        Computes the points for the root fillet on the straight side segment.

        Works by using the tangent vector and solving for lambda which represent how the
        tangent vector has to be scaled to reach the tangent intersection between the
        fillet and the straight line segment. Using this, its also possible to compute
        the flank tangent intersection point and the center of the root fillet.

        Args:
            side (str): Straight segment side.
        """
        points, centers = self._get_side_arrays(side)
        # Solve for lambda first
        s = math.sin(self.partition_angle)
        c = math.cos(self.partition_angle)
        n_h = Vec2(-s, c)
        P = self.pl_trans + self.n_h_trans * self.profile.pld
        P_p = centers[Ctr.FLANK]
        V = P + self.profile.r_root * self.n_h_trans - P_p
        # quadratic root formula
        b = 2 * self.transition_tangent.dot(V)
        c_quad = V.dot(V) - (self.profile.r_root + self.profile.r_flank) ** 2
        dis = b * b - 4 * c_quad
        if dis < 0 and dis > -1e-9:
            dis = 0
        sqrt_val = math.sqrt(dis)
        lambda_1 = (-b + sqrt_val) / 2
        lambda_2 = (-b - sqrt_val) / 2
        p_r1 = P + lambda_1 * self.transition_tangent + self.profile.r_root * self.n_h_trans
        p_r2 = P + lambda_2 * self.transition_tangent + self.profile.r_root * self.n_h_trans
        if side == "left":
            p_r = p_r1 if p_r1.x < p_r2.x else p_r2
        else:
            p_r = p_r1 if p_r1.x > p_r2.x else p_r2
        points[Pt.ROOT_FILL_START] = p_r - n_h * self.profile.r_root
        points[Pt.FLANK_FILL_START] = p_r + ((P_p - p_r).normalized() * self.profile.r_root)
        centers[Ctr.ROOT] = p_r

    def _create_arc(self, p1, p2, center):
        """
        Creates points approximating the shortest arc between two points.

        Args:
            p1: First point defining the arc.
            p2: Second point defining the arc.
            center: Arc center point.

        Returns:
            List of points defining the arc.
        """
        n = 3  # partitions per degree
        min_partitions = 10

        # Calculate the radius
        radius = (p1 - center).norm()
        phi_1 = math.atan2(p1.y - center.y, p1.x - center.x)
        phi_2 = math.atan2(p2.y - center.y, p2.x - center.x)
        delta_phi = phi_2 - phi_1
        # Partitions and shortest arc selection
        if delta_phi > math.pi:
            delta_phi -= 2 * math.pi
        elif delta_phi < -math.pi:
            delta_phi += 2 * math.pi
        k = int(delta_phi * n)
        n_part = max(min_partitions, k)
        step = delta_phi / (n_part - 1) if n_part > 1 else 0
        return [
            Vec2(
                radius * math.cos(phi_1 + step * i) + center.x,
                radius * math.sin(phi_1 + step * i) + center.y,
            )
            for i in range(n_part)
        ]

    def _create_all_arcs(self, side, only_top=False):
        """
        Creates all the arcs for one side of the tooth profile to plot.

        Args:
            side (str): Side to generate arc segments for.
            only_top (bool): Only returns the top arcs. Defaults to False.

        Returns:
            List containing all arcs created.
        """
        if side == "left":
            points, centers = self.global_points_left, self.global_centers_left
        else:
            points, centers = self.global_points_right, self.global_centers_right
        topology = [
            (points[Pt.ROOT_FILL_START], points[Pt.FLANK_FILL_START], centers[Ctr.ROOT]),
            (points[Pt.FLANK_FILL_START], points[Pt.TIP_FILL_START], centers[Ctr.FLANK]),
            (points[Pt.TIP_FILL_START], points[Pt.TIP_END], centers[Ctr.TIP]),
        ]
        side_is_curved = bool(self.D) and (not self.is_transitioning or side == self.opposite_side)
        if side_is_curved:
            topology.append((points[Pt.FLAT_UPPER_CORNER], points[Pt.ROOT_FILL_START], self.global_curve_center))
            if not only_top:
                topology.append((points[Pt.BACK_CENTER], points[Pt.BACK_LOWER_CORNER], self.global_curve_center))
        return [self._create_arc(*arc) for arc in topology]

    def _create_all_straight_line_segments(self, side, only_top=False):
        """
        Creates all the straight line segments for one side of the tooth profile.

        Args:
            side (str): Side to generate line segments for.
            only_top (bool): Only returns the top root flat line segment. Used when
                drawing multiple teeth next to each other. Defaults to False.

        Returns:
            List containing all created line segments.
        """
        points = self.global_points_left if side == "left" else self.global_points_right
        root_flat = (points[Pt.FLAT_UPPER_CORNER], points[Pt.ROOT_FILL_START])
        side_is_curved = bool(self.D) and (not self.is_transitioning or side == self.opposite_side)
        if only_top:
            topology = [] if side_is_curved else [root_flat]
        else:
            topology = [(points[Pt.BACK_LOWER_CORNER], points[Pt.FLAT_UPPER_CORNER])]
            if not side_is_curved:
                topology.extend([(points[Pt.BACK_CENTER], points[Pt.BACK_LOWER_CORNER]), root_flat])
        return [[p1, p2] for p1, p2 in topology]

    def plot_tangent_arrow(self, ax):
        """Plot the tangent and normal vectors for visualization and debugging."""
        if not self.is_transitioning:
            return
        scale_factor = 1 / 3
        start_global = self._transform(self.pl_trans)
        tip_global_tangent = self._transform(self.pl_trans + self.transition_tangent * 2)
        scale = (tip_global_tangent - start_global).norm() / self.profile.pld * scale_factor
        dx_t = (tip_global_tangent.x - start_global.x) / scale
        dy_t = (tip_global_tangent.y - start_global.y) / scale
        ax.arrow(start_global.x, start_global.y, dx_t, dy_t, width=0.015, fc="red", ec="red")

        tip_global_normal = self._transform(self.pl_trans + self.n_h_trans * 2)
        scale = (tip_global_normal - start_global).norm() / self.profile.pld * scale_factor
        dx_n = (tip_global_normal.x - start_global.x) / scale
        dy_n = (tip_global_normal.y - start_global.y) / scale
        ax.arrow(start_global.x, start_global.y, dx_n, dy_n, width=0.015, fc="blue", ec="blue")

    def plot_tooth(self):
        """
        Plot a single tooth with segment points, circle centers and the tangent and
        normal vectors. Good for debugging.

        Returns:
            Matplotlib Figure and Axes objects, or (None, None) if matplotlib is absent.
        """

        try:
            import matplotlib.pyplot as plt
            from matplotlib.collections import LineCollection
        except ImportError:
            print("Notice: Matplotlib is not installed in this environment")
            return None, None

        _, ax = plt.subplots()
        # Plot the segment points and circle centers
        ax.scatter(
            [p.x for p in self.global_points],
            [p.y for p in self.global_points],
            c="red",
            marker="o",
            label="Tooth Profile",
        )
        ax.scatter(
            [p.x for p in self.global_centers],
            [p.y for p in self.global_centers],
            c="blue",
            marker="x",
            label="Circle Centers",
        )
        ax.set_title("Tooth Profile and Circle Centers")
        ax.set_xlabel("$x$")
        ax.set_ylabel("$y$")
        ax.legend()
        ax.grid(True)
        ax.set_aspect("equal", adjustable="box")
        # Plot the tangent and normal vectors
        self.plot_tangent_arrow(ax)

        # Draw all the segments
        segments = self._create_all_arcs("left") + self._create_all_arcs("right")
        segments += self._create_all_straight_line_segments("left")
        segments += self._create_all_straight_line_segments("right")
        drawable_segments = [[(point.x, point.y) for point in segment] for segment in segments]
        ax.add_collection(LineCollection(drawable_segments, linestyle="solid", color="black"))
        ax.autoscale()
        ax.autoscale_view()

        plt.show()