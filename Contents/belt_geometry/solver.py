"""
Belt system solver script.

Reduced version of pybeltsolver.
"""

import math
from enum import Enum

from .replacements import (
    Vec2,
    bisection_method,
    compute_point_distance,
    secant_method,
)


class BeltSolverError(Exception):
    """Base class for all errors raised by the Belt kinematic solver."""


class TopologyMismatchError(BeltSolverError):
    """Raised when the number of faces does not match the number of pulleys."""


class OverlappingPulleyError(BeltSolverError):
    """
    Raised when the distance between two pulleys is less than their combined radii.
    """


class CrossingBeltError(BeltSolverError):
    """Raised when a crossing belt is detected and crossing is not allowed."""


class BeltFace(Enum):
    """
    Defines the face of the belt for routing purposes.
    """

    FRONT = 1
    BACK = 2


class Circle:
    """
    Represents the Belt Pulleys.
    """

    def __init__(self, r: float, coords: tuple[float, float], name: str | None = None) -> None:
        """
        Initializes the Circle class.

        Args:
            r (float): Radius of the circle.
            coords (tuple[float, float]): Circle center coordinates as a 2D tuple [x, y].
            name (str, optional): Name of the circle. Defaults to None.
        """

        self.x = coords[0]
        self.y = coords[1]
        self.coords = coords
        self.r = r
        self.name = name

    def dist(self, c: "Circle") -> float:
        """
        Calculates the distance between two circles (self, other)

        Args:
            c (Circle): The other circle to calculate the distance to.

        Returns:
            float: The distance between the centers of the two circles.
        """

        return compute_point_distance(self.coords, c.coords)

    def __str__(self) -> str:
        """
        String representation of the circle object.

        Returns:
            str: A string representation of the circle object.
        """
        return f"Circle '{self.name}' at ({self.x}, {self.y}) with radius {self.r}"


class Belt:
    """
    Complete representation of a generic belt. Includes geometry calculations,
    validation, optimization methods and visualization.
    """

    def __init__(
        self,
        circles: list[Circle],
        topology: list[BeltFace] | None = None,
        do_validate: bool = True,
        allow_crossing: bool = False,
    ) -> None:
        """
        Validates the user input, computes the geometry and checks for crossing belt
        configurations.

        Args:
            circles (list[Circle]): List of Circle objects representing the pulleys in
                the belt system. Must be ordered sequentially (preferably clockwise) in
                the direction of the belt routing.
            topology (list[BeltFace], optional): List of instructions for how the belt
                is to be routed. Defaults to None, which assumes the front (inside) of
                the belt touches all pulleys.
            do_validate (bool, optional): Runs geometric validation checks upon
                initialization. Defaults to True.
            allow_crossing (bool, optional): If False, throws an error when a crossing
                belt (Figure-8) is detected. Set to True to allow intentional crossings.
                Defaults to False.
        """

        self.circles = circles
        self.topology = topology if topology else [BeltFace.FRONT] * len(circles)
        self.do_validate = do_validate
        self.allow_crossing = allow_crossing
        self.lines = []
        self.arcs = []

        self.line_lengths = []
        self.arcs_lengths = []

        self.total_length = 0.0
        self.wrap_points = {c: {"entry": None, "exit": None} for c in circles}
        self.wrap_angles = {c: {"entry": None, "exit": None} for c in circles}
        self.wrap_lines = {c: {"entry": None, "exit": None} for c in circles}
        self.wrap_line_indices = {c: [] for c in circles}

        self.debug = False

        if self.do_validate:
            self._check_validity()

        self._calculate_geometry()

        self._check_crossing_lines()

    def _check_validity(self) -> None:
        """
        Validates the user input for the belt configuration.

        Raises:
            BeltSolverError: Raised for insufficient number of circles.
            TopologyMismatchError: Raised for topology list and circle list length
                mismatch.
            OverlappingPulleyError: Raised for overlapping pulleys.
        """
        # Basic input error checks
        if len(self.circles) < 2:
            raise BeltSolverError("At least two circles are required to form a belt.")

        if len(self.topology) != len(self.circles):
            raise TopologyMismatchError(
                f"Topology list ({len(self.topology)}) must match the number of "
                f"circles ({len(self.circles)})."
            )
        # Check for overlapping circles
        for i, c1 in enumerate(self.circles):
            for j in range(len(self.circles)):
                if i != j:
                    c2 = self.circles[j]
                    if c1.dist(c2) < (c1.r + c2.r):
                        raise OverlappingPulleyError(
                            f"Circles '{c1.name}' and '{c2.name}' are overlapping."
                        )

    def _check_crossing_lines(self) -> None | bool:
        """
        Checks for crossing belts and raises an error if crossing is not allowed.

        Returns:
            bool: True if crossing, False otherwise.

        Raises:
            CrossingBeltError: Raised when a crossing belt is detected and crossing is
            not allowed.
        """

        num_circles = len(self.circles)

        if num_circles < 3:
            return False  # No crossing possible

        # Check for lines
        for c in self.circles:
            line_in = self.wrap_lines[c]["entry"]
            line_out = self.wrap_lines[c]["exit"]

            P_in = line_in[0]
            P_out = line_out[0]

            V_in = line_in[1] - line_in[0]
            V_out = line_out[1] - line_out[0]

            P = P_out - P_in
            P0, P1 = P.to_list()

            a, c = V_in.to_list()
            b, d = -V_out.x, -V_out.y

            # Determinant will be close to zero for parallel lines
            det = a * d - b * c

            if abs(det) > 1e-9:
                det_x = P0 * d - b * P1
                det_y = a * P1 - P0 * c

                lambd_in = det_x / det
                lambd_out = det_y / det

                if 0 <= lambd_in <= 1 and 0 <= lambd_out <= 1:
                    if not self.allow_crossing:
                        raise CrossingBeltError(
                            f"Fatal Geometry: Belt crosses itself at '{c.name}'."
                            f"If intentional, enable 'Allow Crossing'."
                        )
                    else:
                        if self.debug:
                            print(f"Notice: crossed belt detected at '{c.name}'.")
                            print(lambd_in, lambd_out)
                        return True

        # Check for crossing circles and lines
        for c in self.circles:
            indices = self.wrap_line_indices[c]
            
            count = len(self.lines)
            for i in range(count):
                if i in indices:
                    continue

                line = self.lines[i]

                p0, p1 = line
                P1, P2 = p0.to_list()

                n = p1 - p0  # Not normalized on purpose
                n1, n2 = n.to_list()

                h, k = c.coords
                r = c.r
                discriminant = (
                    -(P1**2) * (n2**2)
                    + 2 * P1 * P2 * n1 * n2
                    + 2 * P1 * h * (n2**2)
                    - 2 * P1 * k * n1 * n2
                    - (P2**2) * (n1**2)
                    - 2 * P2 * h * n1 * n2
                    + 2 * P2 * k * (n1**2)
                    - (h**2) * (n2**2)
                    + 2 * h * k * n1 * n2
                    - (k**2) * (n1**2)
                    + (n1**2) * (r**2)
                    + (n2**2) * (r**2)
                )

            if discriminant < 0:
                continue

            denom = n1**2 + n2**2
            if denom == 0:
                continue

            sqrt_disc = math.sqrt(discriminant)

            base_term = -P1 * n1 - P2 * n2 + h * n1 + k * n2

            lam1 = (base_term - sqrt_disc) / denom
            lam2 = (base_term + sqrt_disc) / denom

            if 0.0 <= lam1 <= 1.0 or 0.0 <= lam2 <= 1.0:
                if not self.allow_crossing:
                    raise CrossingBeltError(
                        f"Fatal Geometry: Belt crosses itself at '{c.name}' with "
                        f"a line segment! If intentional, enable 'Allow Crossing'."
                    )
                else:
                    if self.debug:
                        print(
                            f"Notice: crossed belt detected at '{c.name}' with a "
                            f"line segment!"
                        )
                    return True

            return False

    def _calculate_geometry(self) -> None:
        """
        Generates the geometry of the belt by computing the tangencies and arcs
        appropriate to the specified topology.
        """
        # Clear previous geometry (for solvers)
        self.lines.clear()
        self.arcs.clear()
        self.line_lengths.clear()
        self.arcs_lengths.clear()

        # Tangent lines
        for i in range(len(self.circles)):
            j = (i + 1) % len(self.circles)
            c1 = self.circles[i]
            c2 = self.circles[j]
            c1_routing = self.topology[i]
            c2_routing = self.topology[j]

            # Routing assumes clockwise topology
            if c1_routing == c2_routing:
                which = "R" if c1_routing == BeltFace.FRONT else "L"
                line = self._create_outer_tangent_lines(c1, c2, which=which)
            else:
                which = "RL" if c1_routing == BeltFace.FRONT else "LR"
                line = self._create_inner_tangent_lines(c1, c2, which=which)
            self.lines.append(line)

            # Add entry and exit points and lines to the circles
            self.wrap_points[c1]["exit"] = line[0]
            self.wrap_points[c2]["entry"] = line[1]

            self.wrap_lines[c1]["exit"] = line
            self.wrap_lines[c2]["entry"] = line

            # Associated line indices for cross check
            self.wrap_line_indices[c1].append(i)
            self.wrap_line_indices[c2].append(i)

        for i, c in enumerate(self.circles):
            entry_point = self.wrap_points[c]["entry"]
            exit_point = self.wrap_points[c]["exit"]

            entry_line = self.wrap_lines[c]["entry"]
            exit_line = self.wrap_lines[c]["exit"]

            arc = self._create_arcs(c, entry_point, exit_point, entry_line, exit_line)
            self.arcs.append(arc)

        self._calculate_total_length()

    def _calculate_total_length(self) -> None:
        """
        Calculates the total belt length.
        """
        self.total_length = sum(self.line_lengths) + sum(self.arcs_lengths)

    def find_movable_circle_position(
        self,
        target_length: float,
        movable_circle_idx: int,
        slide_vector: Vec2,
        method: str = "bisection",
    ) -> bool:
        """
        Find the new position of a specified circle along the specified slide vector to
        achieve the target length.

        Args:
            target_length (float): Target length of the belt.
            movable_circle_idx (int): Index of the circle to move.
            slide_vector (Vec2): Vector along which to slide the circle.
            method (str): Choose either a secant method or use the default bisection 
                method. The secant method may find solutions that are not on the line
                defined by the slide vector points, whereas the bisection method will
                only find solutions (if they exist) on this line.

        Returns:
            bool: True if a valid position is found, False otherwise.
        """
        movable_circle = self.circles[movable_circle_idx]
        original_coords = movable_circle.coords

        norm = slide_vector.norm()
        #slide_vector = slide_vector / norm

        def length_error(t):
            orig_vec = Vec2(original_coords[0], original_coords[1])
            new_pos = orig_vec + (t * slide_vector)

            new_pos_tuple = (new_pos.x, new_pos.y)
            movable_circle.coords = new_pos_tuple
            movable_circle.x, movable_circle.y = new_pos.x, new_pos.y

            self._calculate_geometry()

            return self.total_length - target_length

        try:
            if method == "secant":
                slide_vector = slide_vector / norm
                result = secant_method(length_error, x0=0.0, x1=0.1)
            elif method == "bisection":
                result = bisection_method(length_error, 0, 1)
            else:
                raise KeyError(f"Solver method `{method}` is not a valid option!")

            print(f"Position found! Shifted by {result:.2f} units.")
            if self.do_validate:
                self._check_crossing_lines()

            return True

        except ValueError as e:
            print(
                f"Target length is physically impossible on this slide vector."
                f"Error: {e}"
            )
            movable_circle.coords = original_coords
            movable_circle.x, movable_circle.y = original_coords
            self._calculate_geometry()

            return False

    def find_editable_circle_radius(
        self, target_length: float, editable_circle_idx: int
    ) -> bool:
        """
        Find the new radius of a specified circle to achieve the target length.

        Args:
            target_length (float): Target length of the belt.
            editable_circle_idx (int): Index of the circle to adjust.

        Returns:
            bool: True if a valid radius is found, False otherwise.
        """

        editable_circle = self.circles[editable_circle_idx]
        original_radius = editable_circle.r

        def length_error(r):
            editable_circle.r = r
            self._calculate_geometry()
            return self.total_length - target_length

        try:
            result = secant_method(
                length_error, x0=original_radius, x1=original_radius * 1.1
            )
            print(f"Radius found! Adjusted by {result:.2f} units.")
            if self.do_validate:
                self._check_crossing_lines()

            return True

        except ValueError as e:
            print(
                f"Target length is physically impossible with this radius adjustment."
                f"Error: {e}"
            )
            editable_circle.r = original_radius
            self._calculate_geometry()
            return False

    def plot(self, show: bool = True) -> tuple:
        """
        Plots the complete Belt in matplotlib.
        Requires matplotlib to be installed in the running environment.
        """
        try:
            import matplotlib.pyplot as plt
            from matplotlib.collections import LineCollection
        except ImportError:
            print("Notice: Matplotlib is not installed in this environment")
            return None, None

        fig, ax = plt.subplots()

        drawable_lines = []
        for line in self.lines:
            drawable_lines.append([line[0].to_list(), line[1].to_list()])

        drawable_arcs = []
        for arc in self.arcs:
            arc_points = [point.to_list() for point in arc]
            drawable_arcs.append(arc_points)

        segs = drawable_lines + drawable_arcs

        line_segments = LineCollection(segs, linestyle="solid", color="red")
        ax.add_collection(line_segments)

        ax.autoscale()
        ax.set_aspect("equal")
        if show:
            plt.show()

        return fig, ax

    def _create_outer_tangent_lines(
        self, c1: Circle, c2: Circle, which="L"
    ) -> list[Vec2]:
        """
        Computes the outer tangents between two circles on the specified side.

        Args:
            c1 (Circle): First circle.
            c2 (Circle): Second circle.
            which (str, optional): Which side the tangent should be draw on. Side is
                defined by the left and right side of the line between the circle
                centers. Defaults to "L".

        Returns:
            list[Vec2, Vec2]: List containing the coordinates of the tangent line.
        """
        hypotenuse = c1.dist(c2)
        short = c1.r - c2.r

        ratio = max(-1.0, min(1.0, short / hypotenuse))

        if which == "R":
            phi = math.atan2(c2.y - c1.y, c2.x - c1.x) + math.acos(ratio)
        elif which == "L":
            phi = math.atan2(c2.y - c1.y, c2.x - c1.x) - math.acos(ratio)
        else:
            raise ValueError(
                f"Invalid input: 'which' must be 'L' or 'R', got '{which}'."
            )

        t1x = c1.x + c1.r * math.cos(phi)
        t1y = c1.y + c1.r * math.sin(phi)

        t2x = c2.x + c2.r * math.cos(phi)
        t2y = c2.y + c2.r * math.sin(phi)

        line = [Vec2(t1x, t1y), Vec2(t2x, t2y)]

        length = compute_point_distance(line[0].to_list(), line[1].to_list())
        self.line_lengths.append(length)

        return line

    def _create_inner_tangent_lines(
        self, c1: Circle, c2: Circle, which="RL"
    ) -> list[Vec2]:
        """
        Creates the inner (or crossing) tangent line between two circles.

        Args:
            c1 (Circle): First circle.
            c2 (Circle): Second circle.
            which (str, optional): Which side the tangent should be drawn on. See
            outer_tangent_lines for more details on side definition. Defaults to "RL".

        Returns:
            list[Vec2, Vec2]: List containing the coordinates of the tangent line.
        """

        hypotenuse = c1.dist(c2)
        short = c1.r + c2.r

        ratio = max(-1.0, min(1.0, short / hypotenuse))  # Added clip here

        if which == "RL":
            phi = math.atan2(c2.y - c1.y, c2.x - c1.x) - math.asin(ratio) + math.pi / 2
        else:
            phi = math.atan2(c2.y - c1.y, c2.x - c1.x) + math.asin(ratio) - math.pi / 2

        t1x = c1.x + c1.r * math.cos(phi)
        t1y = c1.y + c1.r * math.sin(phi)

        t2x = c2.x + c2.r * math.cos(phi + math.pi)
        t2y = c2.y + c2.r * math.sin(phi + math.pi)

        line = [Vec2(t1x, t1y), Vec2(t2x, t2y)]

        length = compute_point_distance(line[0].to_list(), line[1].to_list())
        self.line_lengths.append(length)

        return line

    def _create_arcs(
        self,
        c: Circle,
        entry_point: Vec2,
        exit_point: Vec2,
        entry_line: Vec2,
        exit_line: Vec2,
    ) -> list[Vec2]:
        """
        This function uses a ray approach to determine which arc should be drawn.

        Args:
            c (Circle): Circle for which the arc is being calculated.
            entry_point (Vec2): Point where the arc starts.
            exit_point (Vec2): Point where the arc ends.
            entry_line (Vec2): Line from which the arc starts.
            exit_line (Vec2): Line from which the arc ends.

        Returns:
            list[Vec2]: List containing the coordinates of the arc.
        """

        # Internal functions
        def calculate_arc_length(phi_1, phi_2, c: Circle):
            sweep_angle = abs(phi_2 - phi_1)
            arc_length = c.r * sweep_angle
            return arc_length

        def get_arc_points(p1, p2, n, c: Circle):
            step = (p2 - p1) / (n - 1)
            thetas = [p1 + i * step for i in range(n)]
            stack = []
            for theta in thetas:
                stack.append(
                    Vec2(c.r * math.cos(theta) + c.x, c.r * math.sin(theta) + c.y)
                )
            return stack

        def swap_angle(phi_1, phi_2):
            if phi_2 > phi_1:
                phi_2 -= 2 * math.pi
            else:
                phi_2 += 2 * math.pi
            return phi_1, phi_2

        phi_1 = math.atan2(entry_point.y - c.y, entry_point.x - c.x)
        phi_2 = math.atan2(exit_point.y - c.y, exit_point.x - c.x)

        self.wrap_angles[c]["entry"] = phi_1
        self.wrap_angles[c]["exit"] = phi_2

        sweep_angle = abs(phi_2 - phi_1)
        arc_length = calculate_arc_length(phi_1, phi_2, c)

        P = exit_point - entry_point
        P0, P1 = P.to_list()

        # Points on circle
        p1 = entry_point
        p2 = exit_point

        # Intersection lines
        line1_coords = entry_line
        line2_coords = exit_line

        # Determine which point sits opposite on the line
        l1 = (
            line1_coords[0]
            if (line1_coords[0] - p1).norm() > (line1_coords[1] - p1).norm()
            else line1_coords[1]
        )
        l2 = (
            line2_coords[0]
            if (line2_coords[0] - p2).norm() > (line2_coords[1] - p2).norm()
            else line2_coords[1]
        )

        # Compute normals
        v1 = l1 - p1
        v2 = l2 - p2
        n1 = v1 / v1.norm()
        n2 = v2 / v2.norm()

        m_a, m_c = n1.x, n1.y
        m_b, m_d = -n2.x, -n2.y

        det = m_a * m_d - m_b * m_c
        if abs(det) > 1e-6:
            det_x = P0 * m_d - m_b * P1
            det_y = m_a * P1 - P0 * m_c

            lambd_1 = det_x / det
            lambd_2 = det_y / det

            # If both lambdas are positive, the intersect is enclosing the small arc,
            # else the intersect is enclosing the large arc
            if lambd_1 > 0 and lambd_2 > 0:
                if sweep_angle < math.pi:
                    phi_1, phi_2 = swap_angle(phi_1, phi_2)
                    arc_length = calculate_arc_length(phi_1, phi_2, c)
            else:
                if sweep_angle > math.pi:
                    phi_1, phi_2 = swap_angle(phi_1, phi_2)
                    arc_length = calculate_arc_length(phi_1, phi_2, c)
        else:
            # Do a test with the mid angle to determine which arc is correct
            mid_phi = (phi_1 + phi_2) / 2.0
            test_vec = Vec2(math.cos(mid_phi), math.sin(mid_phi))
            outgoing_vec = line2_coords[1] - line2_coords[0]

            if test_vec.dot(outgoing_vec) > 0:
                phi_1, phi_2 = swap_angle(phi_1, phi_2)

            arc_length = calculate_arc_length(phi_1, phi_2, c)

        # Finally, compute arc points
        n = max(5, int(arc_length * 180 / math.pi))
        arcs = get_arc_points(phi_1, phi_2, n, c)
        self.arcs_lengths.append(arc_length)

        return arcs
