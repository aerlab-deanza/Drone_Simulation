"""Video 1 — Euler angles, gimbal lock, and quaternions.

Render one scene (preview quality)::

    manim -ql videos/manim/v1_rotations.py S05GimbalLock

Narration: ``videos/narration/v1_rotations.md``. The 3D scenes are drawn
z-up (math convention) for intuition. The project's own rotation matrices
from :mod:`drone_sim.dynamics.plant` drive every drone and gimbal.
"""

from __future__ import annotations

import numpy as np
from manim import (
    DEGREES,
    DOWN,
    LEFT,
    ORIGIN,
    OUT,
    PI,
    RIGHT,
    UP,
    AnimationGroup,
    Arrow,
    Arrow3D,
    Axes,
    Circle,
    Create,
    Dot,
    FadeIn,
    FadeOut,
    GrowArrow,
    Indicate,
    Line,
    Matrix,
    ReplacementTransform,
    SurroundingRectangle,
    Text,
    TransformMatchingTex,
    ValueTracker,
    VGroup,
    VMobject,
    Write,
    always_redraw,
    linear,
    smooth,
)

from common import (
    ACCENT,
    BAD,
    FG,
    GOOD,
    MUTED,
    WARM,
    X_COLOR,
    Y_COLOR,
    Z_COLOR,
    NarratedScene,
    NarratedThreeDScene,
    bullet_list,
    finding_badge,
    label,
    tex,
)
from drone_sim.dynamics.plant import rotation_x, rotation_y, rotation_z
from drone_sim.visualization.gimbal_lock import euler_rates_near_vertical


def zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body-to-world rotation Rz(yaw) Ry(pitch) Rx(roll), from the project code."""
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)


def polyline(points: np.ndarray, color: str, width: float) -> VMobject:
    """One VMobject through ``points`` (much faster to redraw than many Lines)."""
    return VMobject(stroke_color=color, stroke_width=width).set_points_as_corners(points)


def drone_model(rotation: np.ndarray, scale: float = 1.0, center=ORIGIN) -> VGroup:
    """An X-frame quadcopter (arms, rotors, red nose) rotated by ``rotation``."""
    center = np.asarray(center, dtype=float)
    arm = 1.1 * scale
    diag = arm / np.sqrt(2)
    motors = np.array([[diag, diag, 0], [diag, -diag, 0], [-diag, -diag, 0], [-diag, diag, 0]])
    world = (rotation @ motors.T).T + center
    parts = VGroup(polyline(np.array([world[0], world[2]]), FG, 6),
                   polyline(np.array([world[1], world[3]]), FG, 6))
    angles = np.linspace(0, 2 * PI, 25)
    circle = np.stack([0.38 * scale * np.cos(angles), 0.38 * scale * np.sin(angles),
                       np.zeros_like(angles)], axis=1)
    for motor in motors:
        parts.add(polyline((rotation @ (circle + motor).T).T + center, MUTED, 3))
    nose = rotation @ np.array([arm * 0.85, 0, 0]) + center
    parts.add(polyline(np.array([center, nose]), BAD, 8))
    return parts


def body_axes(rotation: np.ndarray, length: float = 1.8, center=ORIGIN) -> VGroup:
    """Body x/y/z axes as lines (red/green/blue) with a dot at each tip."""
    center = np.asarray(center, dtype=float)
    group = VGroup()
    for i, color in enumerate((X_COLOR, Y_COLOR, Z_COLOR)):
        tip = center + length * rotation[:, i]
        group.add(polyline(np.array([center, tip]), color, 4), Dot(tip, radius=0.06, color=color))
    return group


class S01Intro(NarratedThreeDScene):
    def construct(self):
        self.set_camera_orientation(phi=68 * DEGREES, theta=-40 * DEGREES, zoom=1.25)
        t = ValueTracker(0)
        drone = always_redraw(lambda: drone_model(
            zyx(0.9 * np.sin(1.3 * t.get_value()), 0.6 * np.sin(0.9 * t.get_value() + 1),
                0.8 * t.get_value()), scale=1.4))
        self.add(drone)
        self.play(t.animate.set_value(6), run_time=6, rate_func=linear)
        question = label("Which way am I pointing?", size=40)
        self.add_fixed_in_frame_mobjects(question)
        question.to_edge(UP)
        self.play(FadeIn(question), t.animate.set_value(9), run_time=3, rate_func=linear)
        self.play(FadeOut(question), FadeOut(drone), run_time=0.8)
        title = VGroup(label("Euler angles, gimbal lock,", size=50),
                       label("and quaternions", size=50),
                       label("drone_sim theory · video 1", size=26, color=MUTED)).arrange(DOWN)
        self.add_fixed_in_frame_mobjects(title)
        self.play(FadeIn(title, shift=0.2 * UP), run_time=1.2)
        self.wait(5)
        self.play(FadeOut(title))


class S02EulerAngles(NarratedThreeDScene):
    def construct(self):
        self.set_camera_orientation(phi=65 * DEGREES, theta=-50 * DEGREES, zoom=1.1)
        yaw, pitch, roll = ValueTracker(0), ValueTracker(0), ValueTracker(0)
        rotation = lambda: zyx(roll.get_value(), pitch.get_value(), yaw.get_value())  # noqa: E731
        drone = always_redraw(lambda: drone_model(rotation(), scale=1.3))
        axes = always_redraw(lambda: body_axes(rotation(), 2.2))
        self.add(drone, axes)

        heading = label("ZYX Euler angles", size=34, color=MUTED).to_corner(LEFT + UP)
        self.add_fixed_in_frame_mobjects(heading)
        self.play(FadeIn(heading))

        steps = [
            (yaw, 50 * DEGREES, r"\text{1. yaw } \psi \text{ about } z", Z_COLOR, r"R_z(\psi)"),
            (pitch, 30 * DEGREES, r"\text{2. pitch } \theta \text{ about the new } y", Y_COLOR, r"R_y(\theta)"),
            (roll, 35 * DEGREES, r"\text{3. roll } \phi \text{ about the newest } x", X_COLOR, r"R_x(\phi)"),
        ]
        captions = VGroup()
        for tracker, value, caption, color, _ in steps:
            text = tex(caption, size=34, color=color)
            captions.add(text)
        captions.arrange(DOWN, aligned_edge=LEFT, buff=0.25).to_corner(RIGHT + UP, buff=0.5)
        for (tracker, value, *_), text in zip(steps, captions):
            self.add_fixed_in_frame_mobjects(text)
            self.play(FadeIn(text), run_time=0.6)
            self.play(tracker.animate.set_value(value), run_time=2.2)
            self.wait(1.4)

        product = tex(r"R", r"=", r"R_z(\psi)", r"\,", r"R_y(\theta)", r"\,", r"R_x(\phi)", size=48)
        product[2].set_color(Z_COLOR)
        product[4].set_color(Y_COLOR)
        product[6].set_color(X_COLOR)
        product.to_edge(DOWN, buff=1.1)
        self.add_fixed_in_frame_mobjects(product)
        self.play(Write(product), run_time=2)
        ref = label("drone_sim.dynamics.plant.body_to_world_rotation()", size=22, color=ACCENT,
                    font="Consolas").to_edge(DOWN, buff=0.35)
        self.add_fixed_in_frame_mobjects(ref)
        self.play(FadeIn(ref))
        self.play(yaw.animate.set_value(-20 * DEGREES), pitch.animate.set_value(15 * DEGREES),
                  roll.animate.set_value(-25 * DEGREES), run_time=3)
        self.wait(5)
        self.clear_all()


class S03BodyRates(NarratedScene):
    def construct(self):
        self.heading("Body rates vs Euler-angle rates")
        gyro = tex(r"\text{gyro measures } ", r"\boldsymbol\omega", r"=[\omega_x,\omega_y,\omega_z]^\top",
                   r"\;\neq\;", r"[\dot\phi,\dot\theta,\dot\psi]^\top", size=40)
        gyro[1].set_color(ACCENT)
        gyro.shift(2.2 * UP)
        self.play(Write(gyro), run_time=2.5)
        self.wait(2)

        terms = VGroup(
            tex(r"\boldsymbol\omega", r"=", r"\begin{bmatrix}\dot\phi\\0\\0\end{bmatrix}", size=40),
            tex(r"+", r"R_x(\phi)^\top\begin{bmatrix}0\\\dot\theta\\0\end{bmatrix}", size=40),
            tex(r"+", r"R_x(\phi)^\top R_y(\theta)^\top\begin{bmatrix}0\\0\\\dot\psi\end{bmatrix}", size=40),
        ).arrange(RIGHT, buff=0.55).shift(0.2 * UP)
        terms[0][2].set_color(X_COLOR)
        terms[1][1].set_color(Y_COLOR)
        terms[2][1].set_color(Z_COLOR)
        notes = VGroup(
            label("roll: body x directly", 20, X_COLOR),
            label("pitch: through roll", 20, Y_COLOR),
            label("yaw: through pitch and roll", 20, Z_COLOR),
        )
        for note, term in zip(notes, terms):
            note.next_to(term, DOWN, buff=0.35)
        for term, note in zip(terms, notes):
            self.play(Write(term), FadeIn(note), run_time=2.2)
            self.wait(1.6)

        result = tex(r"\boldsymbol\omega", r"=",
                     r"\underbrace{\begin{bmatrix}1&0&-\sin\theta\\0&\cos\phi&\sin\phi\cos\theta\\"
                     r"0&-\sin\phi&\cos\phi\cos\theta\end{bmatrix}}_{W(\phi,\theta)}",
                     r"\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix}", size=42)
        result.shift(1.8 * DOWN)
        self.play(FadeOut(notes), terms.animate.scale(0.75).shift(0.9 * UP), run_time=1)
        self.play(Write(result), run_time=3)
        self.play(Indicate(result[2], color=WARM), run_time=1.5)
        self.wait(2)
        inverse = tex(r"\text{plant needs } [\dot\phi,\dot\theta,\dot\psi]^\top = W^{-1}\boldsymbol\omega",
                      size=34, color=WARM).to_edge(DOWN, buff=0.4)
        self.play(FadeIn(inverse), run_time=1)
        self.wait(5)
        self.clear_all()


class S04RateBlowUp(NarratedScene):
    def construct(self):
        self.heading("Proof 1: the kinematics blow up")
        det = tex(r"\det W", r"=", r"1\cdot\big(\cos\phi\cdot\cos\phi\cos\theta"
                  r"+\sin\phi\cdot\sin\phi\cos\theta\big)", size=40).shift(2.0 * UP)
        det2 = tex(r"\det W", r"=", r"\cos\theta\,(\cos^2\phi+\sin^2\phi)", size=40).move_to(det)
        det3 = tex(r"\det W", r"=", r"\cos\theta", size=48).move_to(det)
        self.play(Write(det), run_time=3)
        self.wait(1.5)
        self.play(TransformMatchingTex(det, det2), run_time=1.5)
        self.wait(1)
        self.play(TransformMatchingTex(det2, det3), run_time=1.2)
        box = SurroundingRectangle(det3, color=BAD)
        self.play(Create(box))
        zero = tex(r"=0 \;\text{ at } \theta = \pm 90^\circ", size=40, color=BAD).next_to(det3, RIGHT)
        self.play(FadeIn(zero))
        self.wait(2)
        group = VGroup(det3, box, zero)
        self.play(group.animate.scale(0.7).to_corner(UP + RIGHT, buff=0.5), run_time=1)

        data = euler_rates_near_vertical(pitch_deg_range=(60.0, 120.0), gap_deg=0.1, n_points=300)
        mask = data["pitch_deg"] < 90.0
        pitch = data["pitch_deg"][mask]
        roll_rate = np.abs(data["euler_rates_deg_s"][mask, 0])
        axes = Axes(x_range=[60, 90, 5], y_range=[0, 18000, 4000], x_length=9, y_length=3.9,
                    axis_config={"color": MUTED, "include_numbers": True, "font_size": 22},
                    tips=False).shift(0.6 * DOWN)
        x_label = label("pitch θ (deg)", 22, MUTED).next_to(axes.x_axis, DOWN, buff=0.45)
        y_label = label("roll rate (deg/s)", 22, MUTED).rotate(PI / 2).next_to(axes.y_axis, LEFT, buff=0.55)
        self.play(Create(axes), FadeIn(x_label), FadeIn(y_label), run_time=1.5)
        curve = axes.plot_line_graph(pitch, roll_rate, add_vertex_dots=False,
                                     line_color=BAD, stroke_width=4)
        self.play(Create(curve), run_time=4, rate_func=smooth)
        tip = Dot(axes.c2p(pitch[-1], roll_rate[-1]), color=WARM)
        tip_label = label(f"{roll_rate[-1]:,.0f} °/s at θ = 89.9°", 26, WARM).next_to(tip, LEFT, buff=0.3)
        given = label("constant body rate ω = [1, 0.2, 0.5] rad/s", 22, ACCENT).next_to(axes, UP, buff=0.1)
        self.play(FadeIn(tip), FadeIn(tip_label), FadeIn(given), run_time=1)
        self.wait(5)
        self.clear_all()


class S05GimbalLock(NarratedThreeDScene):
    def construct(self):
        self.set_camera_orientation(phi=70 * DEGREES, theta=-35 * DEGREES, zoom=0.95)
        yaw, pitch, roll = ValueTracker(0.0), ValueTracker(0.0), ValueTracker(0.0)

        def ring(radius, rotation, normal_plane, color):
            angles = np.linspace(0, 2 * PI, 61)
            if normal_plane == "yz":
                pts = np.stack([0 * angles, radius * np.cos(angles), radius * np.sin(angles)], axis=1)
            elif normal_plane == "xy":
                pts = np.stack([radius * np.cos(angles), radius * np.sin(angles), 0 * angles], axis=1)
            else:  # xz
                pts = np.stack([radius * np.cos(angles), 0 * angles, radius * np.sin(angles)], axis=1)
            return polyline((rotation @ pts.T).T, color, 5)

        def gimbals():
            r_yaw = rotation_z(yaw.get_value())
            r_pitch = r_yaw @ rotation_y(pitch.get_value())
            r_roll = r_pitch @ rotation_x(roll.get_value())
            outer = ring(3.0, r_yaw, "yz", Z_COLOR)
            middle = ring(2.5, r_pitch, "xy", Y_COLOR)
            inner = ring(2.0, r_roll, "xz", X_COLOR)
            yaw_axis = Line(-3.6 * OUT, 3.6 * OUT, color=Z_COLOR, stroke_width=3)
            roll_axis_dir = r_pitch @ np.array([1.0, 0, 0])
            roll_axis = Line(-3.2 * roll_axis_dir, 3.2 * roll_axis_dir, color=X_COLOR, stroke_width=3)
            body = drone_model(r_roll, scale=0.7)
            return VGroup(outer, middle, inner, yaw_axis, roll_axis, body)

        rig = always_redraw(gimbals)
        self.add(rig)
        heading = label("Proof 2: a degree of freedom disappears", 30, MUTED).to_corner(LEFT + UP)
        legend = VGroup(label("yaw gimbal (outer)", 22, Z_COLOR), label("pitch gimbal (middle)", 22, Y_COLOR),
                        label("roll gimbal (inner)", 22, X_COLOR)).arrange(DOWN, aligned_edge=LEFT)
        legend.to_corner(RIGHT + UP)
        self.add_fixed_in_frame_mobjects(heading, legend)
        self.play(FadeIn(heading), FadeIn(legend))
        self.play(yaw.animate.set_value(0.6), run_time=1.6)
        self.play(roll.animate.set_value(0.5), run_time=1.6)
        self.play(yaw.animate.set_value(0.0), roll.animate.set_value(0.0), run_time=1.2)
        self.wait(1)
        self.play(pitch.animate.set_value(PI / 2), run_time=4)
        warn = label("roll axis ∥ yaw axis", 30, BAD).to_corner(LEFT + DOWN, buff=0.6)
        self.add_fixed_in_frame_mobjects(warn)
        self.play(FadeIn(warn))
        self.play(yaw.animate.set_value(0.7), run_time=1.8)
        self.play(roll.animate.set_value(0.7), run_time=1.8)
        same = label("yaw and roll now turn the drone the same way", 26, WARM).to_corner(RIGHT + DOWN, buff=0.6)
        self.add_fixed_in_frame_mobjects(same)
        self.play(FadeIn(same))
        self.wait(2)
        self.play(FadeOut(rig), FadeOut(legend), FadeOut(warn), FadeOut(same), run_time=0.8)

        algebra = VGroup(
            tex(r"R_y(90^\circ)\,R_x(\phi)\,R_y(90^\circ)^\top = R_{-\hat z}(\phi) = R_z(-\phi)", size=38),
            tex(r"R(\phi, 90^\circ, \psi) = R_z(\psi)\,R_z(-\phi)\,R_y(90^\circ)", size=38),
            tex(r"=", r"R_z(\psi-\phi)", r"\,R_y(90^\circ)", size=46),
        ).arrange(DOWN, buff=0.5)
        algebra[2][1].set_color(BAD)
        self.add_fixed_in_frame_mobjects(algebra)
        for line in algebra:
            self.play(Write(line), run_time=2.2)
            self.wait(1.2)
        note = label("only ψ − φ survives", 30, BAD).next_to(algebra, DOWN, buff=0.5)
        self.add_fixed_in_frame_mobjects(note)
        self.play(FadeIn(note))
        self.wait(4)
        self.play(FadeOut(algebra), FadeOut(note), FadeOut(heading))


class S06NoThreeParameter(NarratedScene):
    def construct(self):
        statement = label("No 3-parameter representation of rotations", 38)
        statement2 = label("is free of singularities.", 38)
        source = label("Stuelpnagel, SIAM Review, 1964", 24, MUTED)
        group = VGroup(statement, statement2, source).arrange(DOWN, buff=0.3).shift(1.0 * UP)
        self.play(FadeIn(group, shift=0.2 * UP), run_time=1.5)
        self.wait(3)
        three = tex(r"(\phi,\theta,\psi)", r"\;\longrightarrow\;", r"(w, x, y, z),\;\; w^2+x^2+y^2+z^2=1",
                    size=44).shift(1.4 * DOWN)
        three[0].set_color(BAD)
        three[2].set_color(GOOD)
        self.play(Write(three), run_time=2.5)
        caption = label("four numbers, one constraint", 28, GOOD).next_to(three, DOWN, buff=0.4)
        self.play(FadeIn(caption))
        self.wait(5)
        self.clear_all()


class S07Quaternions(NarratedScene):
    def construct(self):
        self.heading("Unit quaternions")
        definition = tex(r"q", r"=", r"\big(\cos\tfrac{\alpha}{2},\;\sin\tfrac{\alpha}{2}\,\mathbf n\big)",
                         size=48).shift(2.3 * UP)
        self.play(Write(definition), run_time=2.5)
        self.wait(2.5)
        product = tex(r"(w_1,\mathbf v_1)\otimes(w_2,\mathbf v_2) = "
                      r"\big(w_1w_2-\mathbf v_1\!\cdot\!\mathbf v_2,\; w_1\mathbf v_2+w_2\mathbf v_1+",
                      r"\mathbf v_1\times\mathbf v_2", r"\big)", size=34).shift(1.0 * UP)
        product[1].set_color(WARM)
        self.play(Write(product), run_time=3)
        noncomm = tex(r"i \otimes j = k \quad\text{but}\quad j \otimes i = -k"
                      r"\qquad \text{(order matters, like rotations)}", size=32, color=WARM)
        noncomm.next_to(product, DOWN, buff=0.3)
        self.play(FadeIn(noncomm))
        self.wait(3)

        sandwich = tex(r"(0,\mathbf u') = q\otimes(0,\mathbf u)\otimes q^*", size=44).shift(0.9 * DOWN)
        self.play(Write(sandwich), run_time=2)
        self.wait(2)
        rodrigues = tex(r"\mathbf u' = \cos\alpha\,\mathbf u + (1-\cos\alpha)(\mathbf n\!\cdot\!\mathbf u)\,\mathbf n"
                        r" + \sin\alpha\,(\mathbf n\times\mathbf u)", size=38).shift(2.3 * DOWN)
        rodrigues_label = label("Rodrigues' rotation formula", 24, GOOD).next_to(rodrigues, DOWN, buff=0.25)
        arrow = Arrow(sandwich.get_bottom(), rodrigues.get_top(), color=MUTED, buff=0.1)
        half = label("half-angle identities", 20, MUTED).next_to(arrow, RIGHT)
        self.play(GrowArrow(arrow), FadeIn(half))
        self.play(Write(rodrigues), run_time=3)
        self.play(FadeIn(rodrigues_label))
        self.wait(6)
        self.clear_all()


class S08QuaternionKinematics(NarratedScene):
    def construct(self):
        self.heading("Integrating the gyro")
        kinematics = tex(r"\dot q", r"=", r"\tfrac12\,q\otimes(0,\boldsymbol\omega)", size=60).shift(1.8 * UP)
        self.play(Write(kinematics), run_time=2.5)
        good = label("no tan θ, no 1 / cos θ — bounded for every attitude", 28, GOOD)
        good.next_to(kinematics, DOWN, buff=0.4)
        self.play(FadeIn(good))
        self.wait(3)
        exact = tex(r"\tfrac{d}{dt}\|q\|^2 = \langle q, q\otimes(0,\boldsymbol\omega)\rangle"
                    r" = \|q\|^2\langle 1, (0,\boldsymbol\omega)\rangle = 0", size=36).shift(0.3 * DOWN)
        self.play(Write(exact), run_time=2.5)
        self.wait(2)
        step = tex(r"\big\|q + \dot q\,\Delta t\big\| = \|q\|\sqrt{1+\|\boldsymbol\omega\|^2\Delta t^2/4}",
                   r"\;>\;\|q\|", size=36).shift(1.6 * DOWN)
        step[1].set_color(BAD)
        self.play(Write(step), run_time=2.5)
        fix = tex(r"q \leftarrow q/\|q\|", size=40, color=GOOD).shift(2.7 * DOWN)
        self.play(FadeIn(fix, shift=0.1 * UP))
        self.wait(5)
        self.clear_all()


class S09InTheProject(NarratedScene):
    def construct(self):
        self.heading("How drone_sim uses this")
        rows = [
            ("Plant", "Euler angles", "raises an error near ±90°", MUTED),
            ("PID controller", "Euler angles", "tilt limited to ±20°", MUTED),
            ("Madgwick, EKF", "quaternions", "no singularity", GOOD),
            ("Live viewer", "quaternions", "survives the board pointing up", GOOD),
        ]
        table = VGroup()
        for name, rep, why, color in rows:
            row = VGroup(label(name, 26), label(rep, 26, color), label(why, 24, MUTED))
            table.add(row)
        for row in table:
            row[0].set_x(-4.4, LEFT)
        for row in table:
            row[1].move_to([-0.6, 0, 0], aligned_edge=LEFT)
            row[2].move_to([2.0, 0, 0], aligned_edge=LEFT)
        table.arrange(DOWN, buff=0.38, aligned_edge=LEFT).shift(0.9 * UP + 0.3 * LEFT)
        for row in table:
            row[0].set_x(-5.0, LEFT)
            row[1].set_x(-1.7, LEFT)
            row[2].set_x(0.9, LEFT)
        for row in table:
            self.play(FadeIn(row, shift=0.1 * RIGHT), run_time=0.9)
            self.wait(1.4)
        self.wait(1)

        badge = finding_badge(2, "lesson learned").shift(1.6 * DOWN)
        s_matrix = tex(r"\text{NED/FRD} \;\xrightarrow{\;S=\operatorname{diag}(1,-1,-1)\;}\; \text{z-up/FLU},"
                       r"\quad \det S=+1", size=34).next_to(badge, DOWN, buff=0.35)
        self.play(FadeIn(badge))
        self.play(Write(s_matrix), run_time=2.5)
        ref = label("drone_sim/dynamics/frames.py", 22, ACCENT, font="Consolas").to_edge(DOWN, buff=0.3)
        self.play(FadeIn(ref))
        self.wait(5)
        self.clear_all()


class S10Outro(NarratedScene):
    def construct(self):
        points = bullet_list([
            "Euler angles: intuitive, but singular at ±90° pitch",
            "det W = cos θ — the rates blow up",
            "at 90°, only ψ − φ survives — gimbal lock",
            "quaternions: 4 numbers, no singularity, renormalize",
        ], size=30)
        points.shift(0.6 * UP)
        self.play(FadeIn(points, lag_ratio=0.3), run_time=2.5)
        nxt = label("Next: why a still gyroscope drifts", 30, WARM).to_edge(DOWN, buff=1.0)
        self.play(FadeIn(nxt))
        self.wait(5)
        self.clear_all()
