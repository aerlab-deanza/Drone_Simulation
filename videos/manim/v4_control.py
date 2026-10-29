"""Video 4 — Dynamics and control: RK4, PID and LQR.

Render one scene (preview quality)::

    manim -ql videos/manim/v4_control.py S09Hamiltonian

Narration: ``videos/narration/v4_control.md``. Trajectories, eigenvalues
and the saturation numbers come from :mod:`drone_sim` (``project_data.py``).
"""

from __future__ import annotations

import numpy as np
from manim import (
    DOWN,
    LEFT,
    PI,
    RIGHT,
    UP,
    Arrow,
    Axes,
    Circle,
    Create,
    DashedLine,
    Dot,
    FadeIn,
    FadeOut,
    GrowArrow,
    Indicate,
    Line,
    NumberPlane,
    Rectangle,
    RoundedRectangle,
    SurroundingRectangle,
    ValueTracker,
    VGroup,
    VMobject,
    Write,
    always_redraw,
    linear,
)

import project_data as data
from common import (
    ACCENT,
    BAD,
    FG,
    GOOD,
    MUTED,
    PURPLE,
    WARM,
    X_COLOR,
    Y_COLOR,
    Z_COLOR,
    NarratedScene,
    finding_badge,
    label,
    tex,
)


def axes_box(x_range, y_range, x_length=10, y_length=4.5, x_label="", y_label="", numbers=True):
    axes = Axes(x_range=x_range, y_range=y_range, x_length=x_length, y_length=y_length, tips=False,
                axis_config={"color": MUTED, "include_numbers": numbers, "font_size": 22})
    labels = VGroup()
    if x_label:
        labels.add(label(x_label, 22, MUTED).next_to(axes, DOWN, buff=0.15))
    if y_label:
        labels.add(label(y_label, 22, MUTED).rotate(PI / 2).next_to(axes, LEFT, buff=0.15))
    return axes, labels


def side_drone(center, pitch: float, scale: float = 1.0) -> VGroup:
    """Side view of the drone: a bar with two rotors, tilted by ``pitch`` (rad)."""
    direction = np.array([np.cos(pitch), np.sin(pitch), 0.0])
    normal = np.array([-np.sin(pitch), np.cos(pitch), 0.0])
    half = 0.55 * scale
    left, right = center - half * direction, center + half * direction
    body = Line(left, right, color=FG, stroke_width=7)
    rotors = VGroup(*[Line(p + 0.12 * scale * normal - 0.25 * scale * direction,
                           p + 0.12 * scale * normal + 0.25 * scale * direction, color=MUTED, stroke_width=4)
                      for p in (left, right)])
    nose = Dot(right, radius=0.06, color=BAD)
    return VGroup(body, rotors, nose)


class S01Intro(NarratedScene):
    def construct(self):
        recovery = data.lqr_recovery()
        history = recovery["history"][:600]          # first 6 s
        axes, labels = axes_box([-1, 6, 1], [-0.5, 3, 1], x_length=10, y_length=5,
                                x_label="x (m)", y_label="altitude (m)")
        VGroup(axes, labels).shift(0.3 * DOWN)
        self.play(Create(axes), FadeIn(labels), run_time=1)
        target = Dot(axes.c2p(0, 2.0), color=GOOD)   # NED z = 0 is 2 m above the start
        target_tag = label("target", 22, GOOD).next_to(target, RIGHT, buff=0.9).shift(0.5 * UP)
        self.play(FadeIn(target), FadeIn(target_tag))
        k = ValueTracker(0)

        def drone():
            i = int(k.get_value())
            x, _, z = history[i, 0:3]
            # NED z is down; the LQR demo starts at z = +2 m (2 m below the origin), shown as -z up.
            return side_drone(axes.c2p(x, -z + 2.0), history[i, 7], scale=1.0)

        trail = always_redraw(lambda: VMobject(stroke_color=ACCENT, stroke_width=3).set_points_as_corners(
            [axes.c2p(history[j, 0], -history[j, 2] + 2.0) for j in range(0, int(k.get_value()) + 1, 4)]
            or [axes.c2p(history[0, 0], -history[0, 2] + 2.0)] * 2))
        body = always_redraw(drone)
        self.add(trail, body)
        self.play(k.animate.set_value(len(history) - 1), run_time=7, rate_func=linear)
        self.wait(1)
        self.clear_all()
        title = VGroup(label("Dynamics and control", 52), label("RK4, PID and LQR", 32),
                       label("drone_sim theory · video 4", 26, MUTED)).arrange(DOWN)
        self.play(FadeIn(title, shift=0.2 * UP))
        self.wait(4)
        self.play(FadeOut(title))


class S02NewtonEuler(NarratedScene):
    def construct(self):
        self.heading("Newton–Euler equations")
        translation = tex(r"\dot{\mathbf v} = \frac{1}{m}\Big(R\begin{bmatrix}0\\0\\-T\end{bmatrix} - k_d\mathbf v\Big)"
                          r" + \begin{bmatrix}0\\0\\g\end{bmatrix}", size=40).shift(1.9 * UP)
        self.play(Write(translation), run_time=2.5)
        self.wait(2)
        euler = tex(r"\boldsymbol\tau", r"=", r"I\dot{\boldsymbol\omega}", r"+",
                    r"\boldsymbol\omega\times(I\boldsymbol\omega)", size=50).shift(0.1 * DOWN)
        euler[4].set_color(WARM)
        self.play(Write(euler), run_time=2)
        tag = label("gyroscopic term: the body axes rotate", 24, WARM).next_to(euler, DOWN, buff=0.25)
        self.play(Indicate(euler[4], color=WARM), FadeIn(tag))
        self.wait(2)
        components = tex(r"\dot\omega_x = \frac{(I_{yy}-I_{zz})\,\omega_y\omega_z + \tau_x}{I_{xx}},\quad"
                         r"\dot\omega_y = \frac{(I_{zz}-I_{xx})\,\omega_x\omega_z + \tau_y}{I_{yy}},\quad"
                         r"\dot\omega_z = \frac{(I_{xx}-I_{yy})\,\omega_x\omega_y + \tau_z}{I_{zz}}",
                         size=30).to_edge(DOWN, buff=1.0)
        self.play(Write(components), run_time=3)
        ref = label("DronePlant.state_derivative()", 20, ACCENT, font="Consolas").to_edge(DOWN, buff=0.3)
        self.play(FadeIn(ref))
        self.wait(5)
        self.clear_all()


class S03MotorMixing(NarratedScene):
    def construct(self):
        self.heading("Motor mixing (X frame, top view)")
        center = 3.4 * LEFT + 0.3 * DOWN
        arm = 1.9
        positions = {1: (1, -1), 2: (1, 1), 3: (-1, 1), 4: (-1, -1)}   # (forward, right) signs
        frame = VGroup()
        # Screen: forward = up, right = right.
        points = {n: center + arm / np.sqrt(2) * (f * UP + r * RIGHT) for n, (f, r) in positions.items()}
        frame.add(Line(points[1], points[3], color=FG, stroke_width=6), Line(points[2], points[4], color=FG, stroke_width=6))
        spins = {1: "CW", 2: "CCW", 3: "CW", 4: "CCW"}
        for n, p in points.items():
            color = ACCENT if spins[n] == "CW" else WARM
            frame.add(Circle(radius=0.55, color=color, stroke_width=4).move_to(p))
            frame.add(label(f"{n}", 26).move_to(p))
            frame.add(label(spins[n], 18, color).next_to(p, DOWN if n in (3, 4) else UP, buff=0.62))
        nose = Arrow(center, center + 1.2 * UP, color=BAD, buff=0, stroke_width=6)
        frame.add(nose)
        self.play(FadeIn(frame), run_time=1.5)
        lever = VGroup(tex(r"a = \ell/\sqrt2", size=36), label("red arrow: nose (+x)", 20, BAD))
        lever.arrange(RIGHT, buff=0.6).next_to(frame, DOWN, buff=0.3)
        self.play(Write(lever))
        self.wait(1.5)
        cross = tex(r"\mathbf r\times\mathbf F = [r_x, r_y, 0]\times[0,0,-f] = [-r_y f,\; r_x f,\; 0]",
                    size=32).to_corner(UP + RIGHT, buff=0.9).shift(0.3 * DOWN)
        self.play(Write(cross), run_time=2)
        self.wait(2)
        mixer = tex(r"\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix} = "
                    r"\begin{bmatrix}k_t&k_t&k_t&k_t\\ak_t&-ak_t&-ak_t&ak_t\\ak_t&ak_t&-ak_t&-ak_t\\"
                    r"-k_b&k_b&-k_b&k_b\end{bmatrix}\begin{bmatrix}\omega_1^2\\\omega_2^2\\\omega_3^2\\\omega_4^2\end{bmatrix}",
                    size=34).next_to(cross, DOWN, buff=0.5).align_to(cross, RIGHT)
        self.play(Write(mixer), run_time=3)
        yaw = label("CW and CCW pairs → yaw control", 24, WARM).next_to(mixer, DOWN, buff=0.4)
        self.play(FadeIn(yaw))
        self.wait(5)
        self.clear_all()


class S04Saturation(NarratedScene):
    def construct(self):
        badge = finding_badge(10, "the controller asks for impossible torques").to_edge(UP, buff=0.35)
        self.play(FadeIn(badge))
        first = data.first_command()
        requested = np.linalg.norm(first["torques"])
        applied = np.linalg.norm(first["applied_torques"])
        axes, labels = axes_box([0, 3, 1], [0, 4.5, 1], x_length=4.5, y_length=4.2, numbers=False,
                                y_label="|τ| (N·m)")
        VGroup(axes, labels).to_edge(LEFT, buff=1.2).shift(0.4 * DOWN)
        self.play(Create(axes), FadeIn(labels))
        bars = VGroup()
        for x, value, color, name in ((0.9, requested, BAD, "requested"), (2.1, applied, GOOD, "feasible")):
            low, high = axes.c2p(x - 0.35, 0), axes.c2p(x + 0.35, value)
            bar = Rectangle(width=high[0] - low[0], height=high[1] - low[1], fill_color=color,
                            fill_opacity=0.85, stroke_width=0).move_to((low + high) / 2)
            bars.add(VGroup(bar, label(f"{value:.2f}", 22, color).next_to(bar, UP, buff=0.1),
                            label(name, 22, color).next_to(axes.c2p(x, 0), DOWN, buff=0.2)))
        for bar in bars:
            self.play(FadeIn(bar, shift=0.2 * UP), run_time=1)
        ratio = label(f"≈ {requested / applied:.0f}× too much", 26, BAD).next_to(axes, UP, buff=0.4)
        self.play(FadeIn(ratio))
        self.wait(2)
        clip = VGroup(label("naive clipping of each motor:", 24, MUTED),
                      label("12.6 N of thrust → 52 N  (drone shoots up)", 24, BAD)).arrange(DOWN, aligned_edge=LEFT)
        clip.to_corner(RIGHT + UP, buff=0.8).shift(0.9 * DOWN)
        self.play(FadeIn(clip))
        self.wait(2)
        allocation = VGroup(
            tex(r"\mathbf s = \mathbf s_T + \lambda\,\mathbf s_\tau", size=40),
            tex(r"\lambda = \max\{\lambda\in[0,1] : 0 \le s_i \le \omega_{\max}^2\}", size=32),
            label(f"thrust kept at {first['applied_thrust']:.1f} N · torque direction kept", 24, GOOD),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.3).next_to(clip, DOWN, buff=0.6).align_to(clip, LEFT)
        for part in allocation:
            self.play(FadeIn(part), run_time=1.2)
            self.wait(0.8)
        ref = label("DronePlant.allocate_control()", 20, ACCENT, font="Consolas").to_edge(DOWN, buff=0.3)
        self.play(FadeIn(ref))
        self.wait(4)
        self.clear_all()


class S05RK4(NarratedScene):
    def construct(self):
        self.heading("Runge–Kutta 4")
        axes, labels = axes_box([0, 1.2, 0.2], [0.8, 3.6, 0.5], x_length=6.5, y_length=4.6, numbers=False,
                                x_label="t", y_label="y")
        VGroup(axes, labels).to_edge(LEFT, buff=0.9).shift(0.3 * DOWN)
        self.play(Create(axes), FadeIn(labels))
        f = lambda t, y: y  # noqa: E731 — y' = y, exact solution e^t
        exact = axes.plot(lambda t: np.exp(t), x_range=[0, 1.2], color=MUTED, stroke_width=3)
        self.play(Create(exact))
        h, t0, y0 = 1.0, 0.0, 1.0
        k1 = f(t0, y0)
        k2 = f(t0 + h / 2, y0 + h / 2 * k1)
        k3 = f(t0 + h / 2, y0 + h / 2 * k2)
        k4 = f(t0 + h, y0 + h * k3)
        stages = [(t0, y0, k1, ACCENT, "k₁"), (t0 + h / 2, y0 + h / 2 * k1, k2, Y_COLOR, "k₂"),
                  (t0 + h / 2, y0 + h / 2 * k2, k3, PURPLE, "k₃"), (t0 + h, y0 + h * k3, k4, WARM, "k₄")]
        for t, y, slope, color, name in stages:
            dot = Dot(axes.c2p(t, y), color=color)
            seg = Line(axes.c2p(t - 0.15, y - 0.15 * slope), axes.c2p(t + 0.15, y + 0.15 * slope),
                       color=color, stroke_width=5)
            tag = label(name, 24, color).next_to(dot, LEFT if t > 0.6 else UP, buff=0.15)
            self.play(FadeIn(dot), Create(seg), FadeIn(tag), run_time=1.2)
            self.wait(0.6)
        y1 = y0 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        result = Dot(axes.c2p(t0 + h, y1), color=GOOD, radius=0.1)
        self.play(FadeIn(result))
        formula = tex(r"y_{n+1} = y_n + \tfrac h6\,(k_1 + 2k_2 + 2k_3 + k_4)", size=36).to_corner(UP + RIGHT, buff=0.9)
        formula.shift(0.4 * DOWN)
        numbers = label(f"one step of h = 1:  {y1:.4f}  vs  e = {np.e:.4f}", 24, GOOD).next_to(formula, DOWN, buff=0.4)
        order = tex(r"\text{local error } O(h^5),\quad \text{global } O(h^4)", size=32).next_to(numbers, DOWN, buff=0.4)
        self.play(Write(formula), run_time=2)
        self.play(FadeIn(numbers))
        self.play(Write(order), run_time=1.5)
        self.wait(2)
        badge = finding_badge(12, "full trial state in every stage").next_to(order, DOWN, buff=0.6)
        badge.to_edge(RIGHT, buff=0.4)
        lesson = tex(r"k_2 = f\big(t+\tfrac h2,\ \mathbf x + \tfrac h2 k_1\big)\ \text{— including }\boldsymbol\omega",
                     size=26).next_to(badge, DOWN, buff=0.3).to_edge(RIGHT, buff=0.5)
        self.play(FadeIn(badge))
        self.play(Write(lesson), run_time=2)
        self.wait(4)
        self.clear_all()


class S06CascadedPID(NarratedScene):
    def construct(self):
        self.heading("Cascaded PID")
        names = [("position", "50 Hz", ACCENT), ("attitude", "500 Hz", Y_COLOR), ("rate", "every step", WARM)]
        boxes = VGroup()
        for name, rate, color in names:
            box = RoundedRectangle(corner_radius=0.15, width=2.2, height=1.1, color=color)
            boxes.add(VGroup(box, label(name, 26, color).move_to(box).shift(0.17 * UP),
                             label(rate, 20, MUTED).move_to(box).shift(0.25 * DOWN)))
        boxes.arrange(RIGHT, buff=1.9).shift(2.0 * UP + 0.5 * LEFT)
        arrows = VGroup(*[Arrow(boxes[i].get_right(), boxes[i + 1].get_left(), color=MUTED, buff=0.1)
                          for i in range(2)])
        outs = VGroup(label("tilt, thrust", 18, MUTED).next_to(arrows[0], DOWN, buff=0.08),
                      label("rates", 18, MUTED).next_to(arrows[1], DOWN, buff=0.08))
        for i, b in enumerate(boxes):
            self.play(FadeIn(b), run_time=0.8)
            if i < 2:
                self.play(GrowArrow(arrows[i]), FadeIn(outs[i]), run_time=0.6)
        torque = Arrow(boxes[2].get_right(), boxes[2].get_right() + 1.0 * RIGHT, color=MUTED, buff=0.1)
        self.play(GrowArrow(torque), FadeIn(label("torque", 18, MUTED).next_to(torque, DOWN, buff=0.08)))
        self.wait(2)
        derivation = VGroup(
            tex(r"T\cos\theta = mg", size=38),
            tex(r"T\sin\theta = m\,a", size=38),
            tex(r"\Rightarrow\ \theta = \arctan(a/g)", size=42, color=GOOD),
        ).arrange(RIGHT, buff=0.7).shift(0.2 * DOWN)
        for part in derivation:
            self.play(Write(part), run_time=1.4)
            self.wait(0.6)
        thrust = tex(r"T = \frac{m\,(g + a_z)}{\cos\phi\cos\theta}", size=44).shift(1.9 * DOWN)
        note = label("only cos φ cos θ of the thrust points up", 22, MUTED).next_to(thrust, DOWN, buff=0.25)
        self.play(Write(thrust), run_time=1.8)
        self.play(FadeIn(note))
        self.wait(5)
        self.clear_all()


class S07Tracking(NarratedScene):
    def construct(self):
        self.heading("PID on the climbing figure-eight (top view)")
        log = data.pipeline_log("pid")
        pos, ref = log["pos"], log["pos_des"]
        axes, labels = axes_box([-1, 3, 1], [-1.6, 0.8, 0.5], x_length=7.5, y_length=4.5,
                                x_label="x (m)", y_label="y (m)")
        VGroup(axes, labels).to_edge(LEFT, buff=1.0).shift(0.2 * DOWN)
        self.play(Create(axes), FadeIn(labels))
        reference = axes.plot_line_graph(ref[:, 0], ref[:, 1], add_vertex_dots=False, line_color=BAD,
                                         stroke_width=2)
        reference.set_stroke(opacity=0.6)
        self.play(Create(reference), run_time=1.5)
        k = ValueTracker(1)
        flight = always_redraw(lambda: axes.plot_line_graph(
            pos[: int(k.get_value()), 0], pos[: int(k.get_value()), 1], add_vertex_dots=False,
            line_color=ACCENT, stroke_width=4))
        head = always_redraw(lambda: Dot(axes.c2p(*pos[int(k.get_value()) - 1, :2]), color=ACCENT))
        self.add(flight, head)
        self.play(k.animate.set_value(len(pos)), run_time=7, rate_func=linear)
        rmse = np.sqrt(np.mean((pos - ref) ** 2, axis=0))
        table = VGroup(label("RMS error", 26, MUTED),
                       label(f"x  {rmse[0] * 100:.0f} cm", 28), label(f"y  {rmse[1] * 100:.0f} cm", 28),
                       label(f"altitude  {rmse[2] * 100:.1f} cm", 28)).arrange(DOWN, aligned_edge=LEFT)
        table.to_edge(RIGHT, buff=1.0)
        legend = VGroup(label("reference", 22, BAD), label("PID flight", 22, ACCENT)).arrange(DOWN, aligned_edge=LEFT)
        legend.next_to(table, UP, buff=0.8).align_to(table, LEFT)
        self.play(FadeIn(table), FadeIn(legend))
        self.wait(5)
        self.clear_all()


class S08LQRDerivation(NarratedScene):
    def construct(self):
        self.heading("LQR: from cost to Riccati")
        chain = tex(r"\boldsymbol\tau \to \boldsymbol\omega \to \theta \xrightarrow{\times g} \ddot x \to \dot x \to x",
                    size=40).shift(2.4 * UP)
        chain_note = label("near hover: four chains of integrators", 22, MUTED).next_to(chain, DOWN, buff=0.15)
        self.play(Write(chain), FadeIn(chain_note), run_time=2)
        cost = tex(r"J = \int_0^\infty \big(\mathbf x^\top Q\,\mathbf x + \mathbf u^\top R\,\mathbf u\big)\,dt",
                   size=40).shift(1.0 * UP)
        self.play(Write(cost), run_time=2)
        self.wait(1.5)
        hjb = tex(r"0 = \min_{\mathbf u}\big[\mathbf x^\top Q\mathbf x + \mathbf u^\top R\mathbf u"
                  r" + 2\mathbf x^\top P(A\mathbf x + B\mathbf u)\big]", size=36).shift(0.1 * DOWN)
        guess = label("guess V(x) = xᵀPx", 22, MUTED).next_to(hjb, UP, buff=0.12).align_to(hjb, RIGHT)
        self.play(FadeIn(guess), Write(hjb), run_time=2.5)
        self.wait(1.5)
        gain = tex(r"\partial/\partial\mathbf u = 0\ \Rightarrow\ ", r"\mathbf u^\star = -R^{-1}B^\top P\,\mathbf x",
                   size=38).shift(1.2 * DOWN)
        gain[1].set_color(GOOD)
        self.play(Write(gain), run_time=2)
        self.wait(1.5)
        care = tex(r"A^\top P + PA - PBR^{-1}B^\top P + Q = 0", size=46).shift(2.3 * DOWN)
        box = SurroundingRectangle(care, color=WARM, buff=0.15)
        self.play(Write(care), run_time=2)
        self.play(Create(box))
        tag = label("algebraic Riccati equation", 22, WARM).next_to(box, DOWN, buff=0.12)
        self.play(FadeIn(tag))
        self.wait(5)
        self.clear_all()


class S09Hamiltonian(NarratedScene):
    def construct(self):
        self.heading("Solving the Riccati equation")
        hamiltonian = tex(r"\mathcal H = \begin{bmatrix} A & -BR^{-1}B^\top\\ -Q & -A^\top\end{bmatrix}",
                          size=40).to_corner(UP + LEFT, buff=0.9).shift(0.4 * DOWN)
        self.play(Write(hamiltonian), run_time=2)
        proof = VGroup(
            tex(r"\mathcal J\mathcal H \text{ symmetric}", size=32),
            tex(r"\Rightarrow \mathcal H \sim -\mathcal H^\top", size=32),
            tex(r"\Rightarrow \text{eigenvalues } \pm\lambda", size=32, color=GOOD),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.2).next_to(hamiltonian, DOWN, buff=0.4).align_to(hamiltonian, LEFT)
        for part in proof:
            self.play(Write(part), run_time=1.2)
        recovery = data.lqr_recovery()
        eigs = recovery["hamiltonian"]
        plane = NumberPlane(x_range=[-10, 10, 2], y_range=[-4, 4, 2], x_length=6.5, y_length=3.6,
                            background_line_style={"stroke_color": MUTED, "stroke_opacity": 0.25})
        plane.to_edge(RIGHT, buff=0.6).shift(0.6 * UP)
        axis_tags = VGroup(label("Re", 20, MUTED).next_to(plane, RIGHT, buff=0.1),
                           label("Im", 20, MUTED).next_to(plane, UP, buff=0.05))
        self.play(Create(plane), FadeIn(axis_tags), run_time=1)
        dots = VGroup(*[Dot(plane.c2p(np.clip(e.real, -10, 10), np.clip(e.imag, -4, 4)), radius=0.07,
                            color=GOOD if e.real < 0 else BAD) for e in eigs])
        self.play(FadeIn(dots, lag_ratio=0.05), run_time=2)
        mirror = label(f"{len(eigs)} eigenvalues, mirrored about Re = 0", 22, FG).next_to(plane, DOWN, buff=0.2)
        self.play(FadeIn(mirror))
        self.wait(2)
        solution = tex(r"\text{stable eigenvectors } \begin{bmatrix}X_1\\X_2\end{bmatrix}\ \Rightarrow\ ",
                       r"P = X_2X_1^{-1}", size=38).to_edge(DOWN, buff=0.9)
        solution[1].set_color(WARM)
        self.play(Write(solution), run_time=2)
        ref = label("drone_sim.control.lqr.solve_care()", 20, ACCENT, font="Consolas").to_edge(DOWN, buff=0.3)
        self.play(FadeIn(ref))
        self.wait(5)
        self.clear_all()


class S10Limits(NarratedScene):
    def construct(self):
        badge = finding_badge(11, "LQR outside its linearization").to_edge(UP, buff=0.35)
        self.play(FadeIn(badge))
        recovery = data.lqr_recovery()
        t = recovery["t"][:800]
        pitch = np.degrees(recovery["history"][:800, 7])
        axes, labels = axes_box([0, 8, 2], [-30, 60, 30], x_length=5.8, y_length=3.6,
                                x_label="time (s)", y_label="pitch (deg)")
        VGroup(axes, labels).to_edge(LEFT, buff=1.0).shift(0.3 * DOWN)
        self.play(Create(axes), FadeIn(labels))
        graph = axes.plot_line_graph(t, pitch, add_vertex_dots=False, line_color=WARM, stroke_width=4)
        self.play(Create(graph), run_time=2.5)
        peak = int(np.argmax(pitch))
        dot = Dot(axes.c2p(t[peak], pitch[peak]), color=BAD)
        tag = label(f"peak {pitch[peak]:.0f}° — far from hover, still converges", 22, BAD)
        tag.next_to(axes, UP, buff=0.25).align_to(axes, LEFT)
        self.play(FadeIn(dot), FadeIn(tag))
        self.wait(2)
        pid, lqr = data.pipeline_log("pid"), data.pipeline_log("lqr")
        rmse = {name: np.sqrt(np.mean((log["pos"] - log["pos_des"]) ** 2, axis=0)) for name, log in
                (("PID", pid), ("LQR", lqr))}
        rows = VGroup(label("RMS (m)   x     y     alt", 22, MUTED, font="Consolas"))
        for name, values in rmse.items():
            rows.add(label(f"{name:<8}{values[0]:.2f}  {values[1]:.2f}  {values[2]:.2f}", 24,
                           ACCENT if name == "PID" else WARM, font="Consolas"))
        rows.arrange(DOWN, aligned_edge=LEFT, buff=0.2).to_edge(RIGHT, buff=0.6).shift(0.2 * DOWN)
        self.play(FadeIn(rows, lag_ratio=0.3), run_time=1.5)
        why = label("designed to hold a point,\nnot follow a path", 22, MUTED).next_to(rows, DOWN, buff=0.3)
        why.align_to(rows, LEFT)
        self.play(FadeIn(why))
        self.wait(2)
        caveat = label("Caveat: both controllers fly on the true state — flying on the estimates is next.",
                       24, FG).to_edge(DOWN, buff=0.9)
        self.play(FadeIn(caveat))
        self.wait(3)
        self.clear_all()
        end = VGroup(label("Thanks for watching", 44),
                     label("Full derivations: aerlab-deanza.github.io/Drone_Simulation", 28, ACCENT)).arrange(DOWN, buff=0.4)
        self.play(FadeIn(end, shift=0.2 * UP))
        self.wait(4)
        self.play(FadeOut(end))
