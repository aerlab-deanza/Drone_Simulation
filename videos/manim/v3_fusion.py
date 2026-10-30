"""Video 3 - Sensor fusion: Madgwick and the EKF.

Render one scene (preview quality)::

    manim -ql videos/manim/v3_fusion.py S03Objective

Narration: ``videos/narration/v3_fusion.md``. The error surface and descent
path use the project's own Madgwick objective function and filter.
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
    Create,
    Dot,
    Ellipse,
    FadeIn,
    FadeOut,
    Indicate,
    ManimColor,
    Rectangle,
    ReplacementTransform,
    Rotate,
    Square,
    SurroundingRectangle,
    Transform,
    ValueTracker,
    VGroup,
    Write,
    always_redraw,
    interpolate_color,
    linear,
)

import project_data as data
from common import (
    ACCENT,
    BAD,
    BG,
    FG,
    GOOD,
    MUTED,
    PURPLE,
    WARM,
    NarratedScene,
    bullet_list,
    finding_badge,
    label,
    tex,
)
from drone_sim.dynamics.plant import euler_to_quaternion
from drone_sim.estimation.madgwick import MadgwickMARG, default_beta


def axes_box(x_range, y_range, x_length=10, y_length=4.5, x_label="", y_label="", numbers=True):
    axes = Axes(x_range=x_range, y_range=y_range, x_length=x_length, y_length=y_length, tips=False,
                axis_config={"color": MUTED, "include_numbers": numbers, "font_size": 22})
    labels = VGroup()
    if x_label:
        labels.add(label(x_label, 22, MUTED).next_to(axes, DOWN, buff=0.15))
    if y_label:
        labels.add(label(y_label, 22, MUTED).rotate(PI / 2).next_to(axes, LEFT, buff=0.15))
    return axes, labels


def sensor_box(angle: float, color=FG) -> VGroup:
    """A small IMU box with a body 'up' tick, rotated by ``angle`` (rad)."""
    box = Rectangle(width=1.6, height=0.7, color=color, stroke_width=4)
    tick = Arrow(box.get_center(), box.get_center() + 0.9 * UP, color=MUTED, buff=0, stroke_width=3)
    group = VGroup(box, tick)
    group.rotate(angle)
    return group


class S01Intro(NarratedScene):
    def construct(self):
        tilt = data.tilt_correction()
        axes, labels = axes_box([0, 30, 5], [-4, 18, 4], x_length=10, y_length=4.2,
                                x_label="time (s)", y_label="roll (deg)")
        VGroup(axes, labels).shift(0.7 * DOWN)
        self.play(Create(axes), FadeIn(labels), run_time=1)
        rng = np.random.default_rng(2)
        accel_only = np.clip(rng.normal(0, 1.4, len(tilt["t"])), -3.8, 3.8)
        gyro = axes.plot_line_graph(tilt["t"], tilt["only"], add_vertex_dots=False, line_color=BAD,
                                    stroke_width=3)
        accel = axes.plot_line_graph(tilt["t"][::3], accel_only[::3], add_vertex_dots=False,
                                     line_color=PURPLE, stroke_width=1.2)
        fused = axes.plot_line_graph(tilt["t"], tilt["fused"], add_vertex_dots=False, line_color=GOOD,
                                     stroke_width=4)
        tags = VGroup(label("gyro: smooth, but drifts", 24, BAD), label("accelerometer: never drifts, but noisy", 24, PURPLE),
                      label("fused", 24, GOOD)).arrange(DOWN, aligned_edge=LEFT).to_corner(UP + RIGHT, buff=0.4)
        self.play(Create(gyro), FadeIn(tags[0]), run_time=2)
        self.play(Create(accel), FadeIn(tags[1]), run_time=2)
        self.play(Create(fused), FadeIn(tags[2]), run_time=2)
        self.wait(1.5)
        self.clear_all()
        title = VGroup(label("Sensor fusion", 52), label("Madgwick and the extended Kalman filter", 32),
                       label("drone_sim theory · video 3", 26, MUTED)).arrange(DOWN)
        self.play(FadeIn(title, shift=0.2 * UP))
        self.wait(4)
        self.play(FadeOut(title))


class S02SpecificForce(NarratedScene):
    def construct(self):
        self.heading("What an accelerometer measures")
        formula = tex(r"\mathbf f_{\text{body}}", r"=", r"R^\top(\mathbf a - \mathbf g)", size=52).shift(2.3 * UP)
        self.play(Write(formula), run_time=2)
        self.wait(1.5)
        cases = [("on a table", "a = 0", "+g up", GOOD), ("free fall", "a = g", "0", MUTED),
                 ("hovering", "a = 0", "+g up (thrust)", GOOD), ("accelerating, level", "a = [a, 0, 0]", "[a, 0, g]", WARM)]
        table = VGroup()
        for name, accel, reads, color in cases:
            table.add(VGroup(label(name, 24), label(accel, 24, MUTED), label("reads " + reads, 24, color)))
        table.arrange(DOWN, buff=0.22, aligned_edge=LEFT).shift(0.6 * UP)
        for row in table:
            row[0].set_x(-5.2, LEFT)
            row[1].set_x(-1.6, LEFT)
            row[2].set_x(1.2, LEFT)
            self.play(FadeIn(row, shift=0.1 * RIGHT), run_time=0.7)
            self.wait(0.7)
        self.play(FadeOut(table), run_time=0.6)

        badge = finding_badge(5, "tilt or acceleration?").shift(0.8 * UP)
        self.play(FadeIn(badge))
        theta = np.arctan2(3.0, 9.81)
        left = sensor_box(theta, ACCENT).shift(3.2 * LEFT + 1.4 * DOWN)
        right = sensor_box(0, WARM).shift(3.2 * RIGHT + 1.4 * DOWN)
        left_tag = label("still, tilted 17°", 24, ACCENT).next_to(left, DOWN, buff=0.6)
        right_tag = label("level, accelerating 3 m/s²", 24, WARM).next_to(right, DOWN, buff=0.6)
        self.play(FadeIn(left), FadeIn(right), FadeIn(left_tag), FadeIn(right_tag))
        reading_dir = np.array([np.sin(theta), np.cos(theta), 0])   # (a, g) direction
        # Still + tilted: the support force points world-up. Level + accelerating:
        # f = a - g leans forward. Relative to each box's own axes they coincide.
        for box, direction in ((left, UP), (right, reading_dir)):
            arrow = Arrow(box.get_center(), box.get_center() + 1.6 * direction, color=BAD, buff=0,
                          stroke_width=6)
            self.play(Create(arrow), run_time=0.8)
        self.play(right.animate.shift(0.8 * RIGHT), run_time=1.2)
        same = label("same reading in body axes → indistinguishable", 28, BAD).to_edge(DOWN, buff=0.3)
        self.play(FadeIn(same))
        self.wait(4)
        self.clear_all()


class S03Objective(NarratedScene):
    def construct(self):
        self.heading("Madgwick: gradient descent on attitude")
        residual = tex(r"\mathbf f(q)", r"=", r"R(q)^\top\hat z", r"-", r"\hat{\mathbf a}", size=44)
        residual[2].set_color(ACCENT)
        residual[4].set_color(WARM)
        residual.to_corner(UP + RIGHT, buff=1.1).shift(0.4 * DOWN)
        notes = VGroup(label("predicted up", 22, ACCENT), label("measured", 22, WARM))
        notes[0].next_to(residual[2], DOWN, buff=0.15)
        notes[1].next_to(residual[4], DOWN, buff=0.15)
        notes[0].next_to(residual[2], DOWN, buff=0.15).shift(0.3 * LEFT)
        self.play(Write(residual), FadeIn(notes), run_time=2)
        grad = tex(r"\nabla\tfrac12\|\mathbf f\|^2 = J^\top\mathbf f", size=40).next_to(residual, DOWN, buff=0.8)
        self.play(Write(grad), run_time=1.5)

        true_roll, true_pitch = 0.35, -0.25
        q_true = euler_to_quaternion(true_roll, true_pitch, 0.0).to_array()
        w, x, y, z = q_true
        measured_up = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])

        grid = np.linspace(-0.6, 0.6, 31)
        cost = np.zeros((len(grid), len(grid)))
        for i, pitch in enumerate(grid):
            for j, roll in enumerate(grid):
                q = euler_to_quaternion(roll, pitch, 0.0).to_array()
                f = np.array(MadgwickMARG.objective_function(q, 0, 0, measured_up, [0, 0, 0])[:3])
                cost[i, j] = 0.5 * f @ f
        axes, labels = axes_box([-0.6, 0.6, 0.3], [-0.6, 0.6, 0.3], x_length=5.6, y_length=5.6,
                                x_label="roll (rad)", y_label="pitch (rad)")
        VGroup(axes, labels).to_edge(LEFT, buff=1.0).shift(0.4 * DOWN)
        cell = grid[1] - grid[0]
        heat = VGroup()
        normalized = np.sqrt(cost / cost.max())
        for i, pitch in enumerate(grid):
            for j, roll in enumerate(grid):
                low, high = axes.c2p(roll - cell / 2, pitch - cell / 2), axes.c2p(roll + cell / 2, pitch + cell / 2)
                heat.add(Rectangle(width=high[0] - low[0], height=high[1] - low[1], stroke_width=0,
                                   fill_opacity=1, fill_color=interpolate_color(ManimColor(ACCENT), ManimColor(BG), normalized[i, j]))
                         .move_to((low + high) / 2))
        self.play(FadeIn(heat), Create(axes), FadeIn(labels), run_time=1.5)
        target = Dot(axes.c2p(true_roll, true_pitch), color=GOOD, radius=0.1)
        target_tag = label("true tilt", 20, GOOD).next_to(target, RIGHT, buff=0.1)
        self.play(FadeIn(target), FadeIn(target_tag))

        filt = MadgwickMARG(beta=0.5, zeta=0.0)
        path = [(0.0, 0.0)]
        accel = measured_up * 9.81
        for _ in range(140):
            q = filt.update(np.zeros(3), accel, 0.01)
            roll, pitch, _ = filt.quaternion_to_euler(q)
            path.append((roll, pitch))
        points = [axes.c2p(r, p) for r, p in path]
        tracker = ValueTracker(1)
        trail = always_redraw(lambda: VGroup(*[
            Dot(points[k], radius=0.035, color=WARM) for k in range(0, int(tracker.get_value()), 3)]))
        head = always_redraw(lambda: Dot(points[int(tracker.get_value()) - 1], radius=0.08, color=WARM))
        self.add(trail, head)
        self.play(tracker.animate.set_value(len(points)), run_time=5, rate_func=linear)
        tag = label("one small step downhill per sample", 24, WARM).next_to(grad, DOWN, buff=0.6)
        tag.to_edge(RIGHT, buff=0.5)
        self.play(FadeIn(tag))
        ref = label("MadgwickMARG.objective_function()", 20, ACCENT, font="Consolas").to_edge(DOWN, buff=0.25)
        ref.shift(2.8 * RIGHT)
        self.play(FadeIn(ref))
        self.wait(4)
        self.clear_all()


class S04FusedUpdate(NarratedScene):
    def construct(self):
        self.heading("The fused update and β")
        update = tex(r"\dot q", r"=", r"\tfrac12 q\otimes(0,\boldsymbol\omega)", r"-",
                     r"\beta\,\frac{J^\top\mathbf f}{\|J^\top\mathbf f\|}", size=50).shift(2.2 * UP)
        update[2].set_color(ACCENT)
        update[4].set_color(WARM)
        self.play(Write(update), run_time=2.5)
        tags = VGroup(label("follow the gyro", 22, ACCENT).next_to(update[2], DOWN),
                      label("step downhill at speed β", 22, WARM).next_to(update[4], DOWN))
        self.play(FadeIn(tags))
        self.wait(2)
        rule = tex(r"\beta = \Big\|\tfrac12 q\otimes(0,\tilde\omega,\tilde\omega,\tilde\omega)\Big\|"
                   r" = \sqrt{\tfrac34}\,\tilde\omega", size=40).shift(0.2 * UP)
        self.play(Write(rule), run_time=2)
        self.wait(2)
        badge = finding_badge(8, "the 'optimal' β is very slow").shift(1.2 * DOWN)
        beta = default_beta(100.0)
        numbers = tex(rf"\beta \approx {beta * 1e4:.1f}\times10^{{-4}}", r"\;\Rightarrow\;",
                      rf"2\beta \approx {np.degrees(2 * beta):.2f}^\circ/\text{{s}}", r"\;\Rightarrow\;",
                      rf"10^\circ \text{{ takes }} \approx {np.radians(10) / (2 * beta) / 60:.1f}\text{{ min}}",
                      size=36).next_to(badge, DOWN, buff=0.35)
        numbers[4].set_color(BAD)
        self.play(FadeIn(badge))
        self.play(Write(numbers), run_time=2.5)
        fix = label("demo: start from the known attitude  ·  Madgwick's reference code: β = 0.1", 22, MUTED)
        fix.to_edge(DOWN, buff=0.3)
        self.play(FadeIn(fix))
        self.wait(5)
        self.clear_all()


class S05YawUnobservable(NarratedScene):
    def construct(self):
        badge = finding_badge(7, "yaw is unobservable without a magnetometer").to_edge(UP, buff=0.4)
        self.play(FadeIn(badge))
        # Top view: a box spinning about the vertical axis; gravity is a dot (pointing into the page).
        box = Square(side_length=2.4, color=ACCENT, stroke_width=5)
        nose = Arrow(box.get_center(), box.get_center() + 1.6 * RIGHT, color=BAD, buff=0, stroke_width=6)
        drone = VGroup(box, nose).shift(3 * LEFT + 0.3 * DOWN)
        gravity = VGroup(Dot(drone.get_center(), radius=0.12, color=WARM),
                         label("gravity: straight down (into page)", 22, WARM))
        gravity[1].next_to(drone, DOWN, buff=0.4)
        top = label("top view", 22, MUTED).next_to(drone, UP, buff=0.3)
        self.play(FadeIn(drone), FadeIn(gravity), FadeIn(top))
        reading = tex(r"\hat{\mathbf a}_{\text{body}} = [0,\,0,\,1]", size=40).shift(3 * RIGHT + 0.8 * UP)
        residual = tex(r"\mathbf f(q) \text{ unchanged}\ \Rightarrow\ \nabla\mathbf f \text{ has no yaw part}",
                       size=32).next_to(reading, DOWN, buff=0.5)
        self.play(Write(reading))
        self.play(Rotate(drone, angle=PI, about_point=drone.get_center()), run_time=3)
        self.play(Indicate(reading, color=GOOD))
        self.play(Write(residual), run_time=2)
        self.wait(2)
        story = VGroup(label("early version:", 24, MUTED),
                       label("filter heading 0°, drone heading 28.6°", 24),
                       label("→ constant 28.6° yaw error - not a bug", 24, WARM)).arrange(DOWN, aligned_edge=LEFT)
        story.shift(3 * RIGHT + 1.6 * DOWN)
        self.play(FadeIn(story, lag_ratio=0.3), run_time=2)
        self.wait(5)
        self.clear_all()


class S06EKFPredict(NarratedScene):
    def construct(self):
        self.heading("Extended Kalman filter: predict")
        state = tex(r"\mathbf x = [\,\mathbf p,\ \mathbf v,\ q,\ \mathbf b_g,\ \mathbf b_a\,]\in\mathbb R^{16}",
                    size=42).shift(2.5 * UP)
        self.play(Write(state), run_time=2)
        equations = VGroup(
            tex(r"q \leftarrow \operatorname{normalize}\big(q + \tfrac12 q\otimes(0,\tilde{\boldsymbol\omega}-\mathbf b_g)\Delta t\big)", size=32),
            tex(r"\mathbf a = R(q)(\tilde{\mathbf f}-\mathbf b_a) - [0,0,g]^\top", size=32),
            tex(r"\mathbf p \leftarrow \mathbf p + \mathbf v\Delta t + \tfrac12\mathbf a\Delta t^2,\quad"
                r"\mathbf v \leftarrow \mathbf v + \mathbf a\Delta t", size=32),
            tex(r"P \leftarrow F P F^\top + Q", size=38, color=WARM),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.3).shift(0.5 * UP + 1.6 * LEFT)
        for eq in equations:
            self.play(Write(eq), run_time=1.6)
            self.wait(0.8)
        ellipse = Ellipse(width=0.6, height=0.4, color=WARM, fill_opacity=0.25).shift(4.6 * RIGHT + 1.6 * DOWN)
        center = Dot(ellipse.get_center(), color=FG)
        tag = label("uncertainty", 22, WARM).next_to(ellipse, UP, buff=1.3)
        self.play(FadeIn(ellipse), FadeIn(center), FadeIn(tag))
        for _ in range(3):
            self.play(ellipse.animate.scale(1.45).rotate(0.15), center.animate.shift(0.15 * RIGHT),
                      run_time=1.1)
        grows = label("every prediction: uncertainty grows", 26, WARM).to_edge(DOWN, buff=0.35)
        self.play(FadeIn(grows))
        self.wait(4)
        self.clear_all()


class S07EKFUpdate(NarratedScene):
    def construct(self):
        self.heading("Extended Kalman filter: correct")
        gain = tex(r"K = P H^\top", r"(H P H^\top + R_z)^{-1}", size=46).shift(2.4 * UP)
        self.play(Write(gain), run_time=2)
        correction = tex(r"\mathbf x \leftarrow \mathbf x + K\big(\mathbf z - h(\mathbf x)\big)", size=40)
        correction.next_to(gain, DOWN, buff=0.35)
        self.play(Write(correction), run_time=1.6)
        self.wait(1.5)
        ellipse = Ellipse(width=4.0, height=1.6, color=WARM, fill_opacity=0.25).rotate(0.3).shift(0.6 * DOWN)
        self.play(FadeIn(ellipse))
        measured = Arrow(ellipse.get_center() + 1.5 * DOWN + 0.6 * LEFT, ellipse.get_center() + 1.5 * UP + 0.6 * RIGHT,
                         color=ACCENT, buff=0, stroke_width=3)
        seen = label("direction the measurement sees", 22, ACCENT).next_to(measured, RIGHT, buff=0.1)
        self.play(Create(measured), FadeIn(seen))
        squeezed = Ellipse(width=4.0, height=0.5, color=GOOD, fill_opacity=0.3).rotate(0.3).move_to(ellipse)
        self.play(Transform(ellipse, squeezed), run_time=2)
        note = label("uncertainty shrinks only where the measurement can see", 24, GOOD)
        note.to_edge(DOWN, buff=0.9)
        self.play(FadeIn(note))
        self.wait(1.5)
        joseph = tex(r"P \leftarrow \underbrace{(I-KH)P(I-KH)^\top}_{\succeq 0} + \underbrace{K R_z K^\top}_{\succeq 0}",
                     size=34).to_edge(DOWN, buff=0.15)
        self.play(FadeOut(note), Write(joseph), run_time=2)
        self.wait(5)
        self.clear_all()


class S08Observability(NarratedScene):
    def construct(self):
        badge = finding_badge(6, "IMU-only position is unobservable").to_edge(UP, buff=0.35)
        self.play(FadeIn(badge))
        rows = [("roll, pitch", "gravity", "yes (when gravity-valid)", GOOD),
                ("yaw", "magnetometer only", "only with a magnetometer", WARM),
                ("position, velocity", "nothing", "never - H columns are zero", BAD)]
        table = VGroup(*[VGroup(label(a, 24), label(b, 24, MUTED), label(c, 24, color)) for a, b, c, color in rows])
        table.arrange(DOWN, buff=0.22, aligned_edge=LEFT).shift(1.9 * UP)
        for row in table:
            row[0].set_x(-5.6, LEFT)
            row[1].set_x(-2.4, LEFT)
            row[2].set_x(0.7, LEFT)
            self.play(FadeIn(row, shift=0.1 * RIGHT), run_time=0.8)
            self.wait(1.0)
        log = data.pipeline_log("pid")
        error = np.linalg.norm(log["ekf_pos"] - log["pos"], axis=1) * 100
        axes, labels = axes_box([0, 7, 1], [0, 7, 1], x_length=9, y_length=2.9,
                                x_label="time (s)", y_label="EKF error (cm)")
        VGroup(axes, labels).shift(1.35 * DOWN)
        self.play(Create(axes), FadeIn(labels))
        graph = axes.plot_line_graph(log["t"], error, add_vertex_dots=False, line_color=BAD, stroke_width=4)
        self.play(Create(graph), run_time=3)
        tag = label(f"{error[-1]:.1f} cm after 7 s, and accelerating", 24, BAD).next_to(axes.c2p(3.5, 5.5), UP)
        self.play(FadeIn(tag))
        fix = label("needs an outside reference: GPS, camera, optical flow", 22, MUTED)
        fix.next_to(axes.c2p(0.3, 6.5), RIGHT, buff=0).shift(0.6 * DOWN)
        self.play(FadeIn(fix))
        self.wait(4)
        self.clear_all()


class S09GravityValid(NarratedScene):
    def construct(self):
        self.heading("Rules that keep the estimate honest")
        rules = bullet_list([
            "gravity_valid = False during the whole offline flight",
            "live demo: gravity_valid only while the board is still",
            "fast turns: trust the references less",
        ], size=28).shift(1.8 * UP)
        self.play(FadeIn(rules, lag_ratio=0.3), run_time=2.5)
        inflation = tex(r"\sigma_{\text{ref}} \to \sigma_{\text{ref}}\big(1 + (\|\boldsymbol\omega\|/0.45)^2\big)",
                        size=36).next_to(rules, DOWN, buff=0.4)
        self.play(Write(inflation), run_time=1.6)
        self.wait(2)
        badge = finding_badge(9, "lesson learned on real hardware").shift(1.2 * DOWN)
        story = tex(r"\text{library scale } \tfrac{8}{32768}\ (\pm 8g)\ \text{ vs chip set to } \pm 2g"
                    r"\ \Rightarrow\ \text{readings} \times 4", size=32).next_to(badge, DOWN, buff=0.3)
        fixed = label("firmware × 2/8  ·  viewer checks for 1 g before calibrating", 24, GOOD)
        fixed.next_to(story, DOWN, buff=0.3)
        self.play(FadeIn(badge))
        self.play(Write(story), run_time=2.2)
        self.play(FadeIn(fixed))
        self.wait(5)
        self.clear_all()


class S10Outro(NarratedScene):
    def construct(self):
        points = bullet_list([
            "accelerometer = specific force: tilt or acceleration?",
            "Madgwick: one gradient step per sample, speed β",
            "EKF: tracks uncertainty; corrects only what it can see",
            "position and (without a magnetometer) yaw are unobservable",
        ], size=28).shift(0.5 * UP)
        self.play(FadeIn(points, lag_ratio=0.3), run_time=2.5)
        nxt = label("Next: dynamics and control", 30, WARM).to_edge(DOWN, buff=1.0)
        self.play(FadeIn(nxt))
        self.wait(4)
        self.clear_all()
