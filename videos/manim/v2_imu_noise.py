"""Video 2 - IMU noise and drift.

Render one scene (preview quality)::

    manim -ql videos/manim/v2_imu_noise.py S05RandomWalk

Narration: ``videos/narration/v2_imu_noise.md``. Every curve is computed
with :mod:`drone_sim` (see ``project_data.py``).
"""

from __future__ import annotations

import numpy as np
from manim import (
    DOWN,
    LEFT,
    RIGHT,
    UP,
    Axes,
    BarChart,
    Create,
    DashedLine,
    Dot,
    FadeIn,
    FadeOut,
    Indicate,
    ReplacementTransform,
    SurroundingRectangle,
    Transform,
    TransformMatchingTex,
    ValueTracker,
    VGroup,
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
    NarratedScene,
    finding_badge,
    label,
    tex,
)


def axes_box(x_range, y_range, x_length=10, y_length=4.5, x_label="", y_label="", numbers=True):
    """Axes with muted styling and optional labels."""
    axes = Axes(x_range=x_range, y_range=y_range, x_length=x_length, y_length=y_length, tips=False,
                axis_config={"color": MUTED, "include_numbers": numbers, "font_size": 22})
    labels = VGroup()
    if x_label:
        labels.add(label(x_label, 22, MUTED).next_to(axes, DOWN, buff=0.15))
    if y_label:
        labels.add(label(y_label, 22, MUTED).rotate(np.pi / 2).next_to(axes, LEFT, buff=0.15))
    return axes, labels


class S01Intro(NarratedScene):
    def construct(self):
        drift = data.gyro_drift()
        t, angle = drift["t"], drift["angles"][4]
        axes, labels = axes_box([0, 60, 10], [-0.08, 0.08, 0.04], y_label="angle (deg)",
                                x_label="time (s)")
        caption = label("motionless MPU6050 - integrated gyro", 26, MUTED).next_to(axes, UP)
        self.play(Create(axes), FadeIn(labels), FadeIn(caption), run_time=1.2)
        progress = ValueTracker(1)
        curve = always_redraw(lambda: axes.plot_line_graph(
            t[: int(progress.get_value())], angle[: int(progress.get_value())],
            add_vertex_dots=False, line_color=WARM, stroke_width=3))
        self.add(curve)
        self.play(progress.animate.set_value(len(t)), run_time=6, rate_func=linear)
        question = label("Why does a still sensor drift?", 36, FG).to_edge(UP)
        self.play(FadeIn(question))
        self.wait(2)
        self.clear_all()
        title = VGroup(label("IMU noise and drift", 52),
                       label("drone_sim theory · video 2", 26, MUTED)).arrange(DOWN)
        self.play(FadeIn(title, shift=0.2 * UP))
        self.wait(4)
        self.play(FadeOut(title))


class S02ErrorModel(NarratedScene):
    def construct(self):
        self.heading("The error model")
        model = tex(r"\tilde y(t)", r"=", r"y(t)", r"+", r"b_0", r"+", r"b(t)", r"+", r"n(t)", size=56)
        model.shift(2.4 * UP)
        self.play(Write(model), run_time=2)
        parts = [(2, "true value", FG), (4, "constant bias - calibration removes it", ACCENT),
                 (6, "bias instability - wanders slowly", PURPLE), (8, "white noise - new every sample", WARM)]
        notes = VGroup()
        for index, text, color in parts:
            model[index].set_color(color)
            note = label(text, 26, color)
            notes.add(note)
        notes.arrange(DOWN, aligned_edge=LEFT, buff=0.22).shift(0.85 * UP)
        for (index, *_), note in zip(parts, notes):
            self.play(Indicate(model[index], color=note.color), FadeIn(note, shift=0.1 * RIGHT),
                      run_time=1.3)
            self.wait(1.4)

        header = VGroup(label("MPU6050 gyro (z)", 24, MUTED), label("value", 24, MUTED))
        rows = [("noise density N", "6.05e-5 rad/s/√Hz", WARM),
                ("bias instability B", "1.46e-5 rad/s", PURPLE),
                ("correlation time τ", "200 s", PURPLE)]
        table = VGroup(header, *[VGroup(label(a, 24), label(b, 24, c)) for a, b, c in rows])
        for row in table:
            row[1].next_to(row[0], RIGHT, buff=0.6)
        table.arrange(DOWN, aligned_edge=LEFT, buff=0.18)
        for row in table:
            row[1].set_x(1.6, LEFT)
        table.shift(2.1 * DOWN)
        source = label("Gonzalez & Dabove, 2019 - measured on real MPU6050 units", 20, MUTED)
        source.next_to(table, DOWN, buff=0.25)
        self.play(FadeIn(table), FadeIn(source), run_time=1.2)
        self.wait(6)
        self.clear_all()


class S03WhyGaussian(NarratedScene):
    def construct(self):
        from drone_sim.visualization.statistics_demo import sample_means

        self.heading("Why Gaussian? The central limit theorem")
        rng = np.random.default_rng(1)
        axes, labels = axes_box([0, 3, 0.5], [0, 4.5, 1], x_length=9, y_length=4.2, numbers=True)
        VGroup(axes, labels).shift(0.4 * DOWN)
        self.play(Create(axes), run_time=1)
        bins = np.linspace(0, 3, 46)
        width = bins[1] - bins[0]

        def histogram(n):
            means = sample_means("exponential", n, n_trials=20000, rng=rng)
            density, _ = np.histogram(means, bins=bins, density=True)
            bars = VGroup()
            for left, height in zip(bins[:-1], density):
                height = min(height, 4.5)
                if height <= 0:
                    continue
                corner_low = axes.c2p(left, 0)
                corner_high = axes.c2p(left + width, height)
                from manim import Rectangle
                bars.add(Rectangle(width=corner_high[0] - corner_low[0], height=corner_high[1] - corner_low[1],
                                   fill_color=ACCENT, fill_opacity=0.75, stroke_width=0)
                         .move_to((corner_low + corner_high) / 2))
            return bars

        caption = label("means of N exponential samples", 26, MUTED).next_to(axes, UP, buff=0.2)
        self.play(FadeIn(caption))
        current = histogram(1)
        n_label = label("N = 1", 34, WARM).to_corner(RIGHT + UP, buff=0.8)
        self.play(FadeIn(current), FadeIn(n_label))
        self.wait(2)
        for n in (2, 5, 30):
            new = histogram(n)
            new_label = label(f"N = {n}", 34, WARM).move_to(n_label)
            self.play(Transform(current, new), Transform(n_label, new_label), run_time=1.6)
            self.wait(1.6)
        sigma = 1 / np.sqrt(30)
        bell = axes.plot(lambda x: np.exp(-0.5 * ((x - 1) / sigma) ** 2) / (sigma * np.sqrt(2 * np.pi)),
                         x_range=[0.2, 1.8], color=BAD, stroke_width=4)
        self.play(Create(bell), run_time=1.5)
        note = label("sum of many small independent errors → Gaussian", 26, GOOD).to_edge(DOWN, buff=0.4)
        self.play(FadeIn(note))
        self.wait(5)
        self.clear_all()


class S04DensityToSamples(NarratedScene):
    def construct(self):
        self.heading("From noise density to samples")
        formula = tex(r"\sigma_n", r"=", r"N\sqrt{f_s}", size=60).shift(2.2 * UP)
        self.play(Write(formula), run_time=1.5)
        self.wait(1.5)
        rng = np.random.default_rng(3)
        streams = VGroup()
        for row, (rate, color) in enumerate(((100, ACCENT), (400, WARM))):
            axes, labels = axes_box([0, 1, 0.25], [-3, 3, 1], x_length=10, y_length=1.6, numbers=False)
            VGroup(axes, labels).shift((0.1 + 2.1 * row) * DOWN)
            n = rate
            values = rng.standard_normal(n) * np.sqrt(rate / 100) * 1.0
            values = np.clip(values, -3, 3)
            graph = axes.plot_line_graph(np.linspace(0, 1, n), values, add_vertex_dots=False,
                                         line_color=color, stroke_width=1.5)
            tag = label(f"{rate} Hz: σ = N·√{rate}", 24, color).next_to(axes, UP, buff=0.05).align_to(axes, LEFT)
            streams.add(VGroup(axes, graph, tag))
        for stream in streams:
            self.play(Create(stream[0]), FadeIn(stream[2]), run_time=0.6)
            self.play(Create(stream[1]), run_time=1.8)
            self.wait(1.2)
        note = label("4× the rate → 2× noisier samples, same noise per second", 26, GOOD)
        note.to_edge(DOWN, buff=0.25)
        self.play(FadeIn(note))
        value = tex(r"\text{MPU6050 gyro at 100 Hz: } \sigma_n \approx 6.6\times10^{-4}\ \text{rad/s}",
                    size=32, color=FG).next_to(formula, DOWN, buff=0.25)
        self.play(FadeIn(value))
        self.wait(6)
        self.clear_all()


class S05RandomWalk(NarratedScene):
    def construct(self):
        self.heading("Integrating white noise: a random walk")
        steps = VGroup(
            tex(r"\operatorname{Var}\big(\delta\theta(t)\big)", r"=", r"n\,\sigma_n^2\,\Delta t^2", size=40),
            tex(r"=", r"(t f_s)(N^2 f_s)\tfrac{1}{f_s^2}", size=40),
            tex(r"=", r"N^2\,t", size=46),
        ).arrange(RIGHT, buff=0.25).shift(2.4 * UP)
        for part in steps:
            self.play(Write(part), run_time=1.6)
            self.wait(0.8)
        result = tex(r"\sigma_\theta(t) = N\sqrt{t}", size=50, color=WARM).next_to(steps, DOWN, buff=0.3)
        self.play(Write(result))
        self.wait(1.5)

        drift = data.gyro_drift(duration_s=60.0)
        t = drift["t"][:700]           # first 7 s at 100 Hz
        walks = drift["angles"][:40, :700]
        axes, labels = axes_box([0, 7, 1], [-0.03, 0.03, 0.01], x_length=10, y_length=3.6,
                                x_label="time (s)", y_label="angle (deg)")
        VGroup(axes, labels).shift(1.4 * DOWN)
        self.play(Create(axes), FadeIn(labels), run_time=1)
        progress = ValueTracker(2)

        def draw_walks():
            k = int(progress.get_value())
            return VGroup(*[axes.plot_line_graph(t[:k], walk[:k], add_vertex_dots=False,
                                                 line_color=ACCENT, stroke_width=1.2, stroke_opacity=0.5)
                            for walk in walks])

        lines = always_redraw(draw_walks)
        self.add(lines)
        self.play(progress.animate.set_value(len(t)), run_time=5, rate_func=linear)
        lines.clear_updaters()
        n_density = float(drift["N"])
        envelope = VGroup(*[axes.plot(lambda x, s=s: s * np.degrees(n_density * np.sqrt(x)),
                                      x_range=[0.01, 7], color=WARM, stroke_width=4) for s in (1, -1)])
        self.play(Create(envelope), run_time=1.5)
        value = np.degrees(n_density * np.sqrt(7))
        callout = label(f"7 s: N√t = {value:.3f}°  (simulated spread: "
                        f"{np.std(drift['angles'][:, 699]):.4f}°)", 26, WARM).to_edge(DOWN, buff=0.25)
        self.play(FadeIn(callout))
        self.wait(5)
        self.clear_all()


class S06GaussMarkov(NarratedScene):
    def construct(self):
        self.heading("Bias instability: a Gauss-Markov process")
        recursion = tex(r"b_{k+1}", r"=", r"\varphi\, b_k", r"+", r"q\, w_k", r",\qquad",
                        r"\varphi = e^{-\Delta t/\tau_c}", size=44).shift(2.4 * UP)
        self.play(Write(recursion), run_time=2)
        self.wait(2)
        sample = data.gauss_markov_example()
        axes, labels = axes_box([0, 60, 10], [-3, 3, 1], x_length=10, y_length=2.4,
                                x_label="time (s)", y_label="bias / B")
        VGroup(axes, labels).shift(0.5 * UP + 0.0 * DOWN)
        self.play(Create(axes), FadeIn(labels))
        graph = axes.plot_line_graph(sample["t"], np.clip(sample["b"], -3, 3), add_vertex_dots=False,
                                     line_color=PURPLE, stroke_width=2)
        self.play(Create(graph), run_time=3)
        self.wait(1)
        proof = VGroup(
            tex(r"\text{steady state: }\ \sigma^2 = \varphi^2\sigma^2 + q^2", size=36),
            tex(r"\Rightarrow\ \sigma^2 = \frac{q^2}{1-\varphi^2}", size=36),
            tex(r"q = B\sqrt{1-\varphi^2}", r"\ \Rightarrow\ \sigma = B", size=40),
        ).arrange(RIGHT, buff=0.35).scale(0.85).to_edge(DOWN, buff=0.75)
        for part in proof:
            self.play(Write(part), run_time=1.8)
            self.wait(1.2)
        self.play(Create(SurroundingRectangle(proof[2], color=GOOD)))
        ref = label("drone_sim.sensors.imu_simulator.gauss_markov_bias()", 20, ACCENT,
                    font="Consolas").to_edge(DOWN, buff=0.15)
        self.play(FadeIn(ref))
        self.wait(4)
        self.clear_all()


class S07TwoRegimes(NarratedScene):
    def construct(self):
        self.heading("Two drift regimes")
        laws = tex(r"\underbrace{N\sqrt t}_{\text{white noise}}", r"\quad\text{vs}\quad",
                   r"\underbrace{B\,t}_{\text{bias}}", r"\quad\Rightarrow\quad",
                   r"\sigma_\theta \approx \sqrt{N^2 t + (B t)^2}", size=38).shift(2.5 * UP)
        laws[0].set_color(BAD)
        laws[2].set_color(PURPLE)
        laws[4].set_color(GOOD)
        self.play(Write(laws), run_time=3)
        self.wait(1.5)
        drift = data.gyro_drift()
        t = drift["t"]
        n_density, bias_std = float(drift["N"]), float(drift["B"])
        spread = drift["angles"].std(axis=0)
        axes, labels = axes_box([0, 60, 10], [0, 0.06, 0.02], x_length=10, y_length=4.2,
                                x_label="time (s)", y_label="1σ angle error (deg)")
        VGroup(axes, labels).shift(0.9 * DOWN)
        self.play(Create(axes), FadeIn(labels))
        measured = axes.plot_line_graph(t, spread, add_vertex_dots=False, line_color=FG, stroke_width=4)
        white = axes.plot(lambda x: np.degrees(n_density * np.sqrt(x)), x_range=[0.01, 60], color=BAD)
        both = axes.plot(lambda x: np.degrees(np.sqrt(n_density ** 2 * x + (bias_std * x) ** 2)),
                         x_range=[0.01, 60], color=GOOD)
        self.play(Create(measured), run_time=2.5)
        legend = VGroup(label("200 simulated runs", 22, FG), label("N√t", 22, BAD),
                        label("√(N²t + (Bt)²)", 22, GOOD)).arrange(DOWN, aligned_edge=LEFT)
        legend.next_to(axes.c2p(2, 0.06), DOWN + RIGHT, buff=0.1)
        self.play(FadeIn(legend[0]))
        self.play(Create(white), FadeIn(legend[1]), run_time=1.5)
        self.wait(1)
        self.play(Create(both), FadeIn(legend[2]), run_time=1.5)
        crossover = n_density ** 2 / bias_std ** 2
        line = DashedLine(axes.c2p(crossover, 0), axes.c2p(crossover, 0.06), color=MUTED)
        tag = label(f"t = N²/B² ≈ {crossover:.0f} s", 24, WARM).next_to(line, UP, buff=0.05)
        self.play(Create(line), FadeIn(tag))
        self.wait(5)
        self.clear_all()


class S08IntegrationError(NarratedScene):
    def construct(self):
        badge = finding_badge(4, "integration error beats sensor noise").to_edge(UP, buff=0.35)
        self.play(FadeIn(badge))
        errors = data.pipeline_errors()
        t = errors["t"]
        axes, labels = axes_box([0, 7, 1], [-0.18, 0.18, 0.06], x_length=10, y_length=3.6,
                                x_label="time (s)", y_label="roll error (deg)")
        VGroup(axes, labels).shift(0.25 * DOWN)
        self.play(Create(axes), FadeIn(labels))
        noisy = axes.plot_line_graph(t, errors["noisy"], add_vertex_dots=False,
                                     line_color=WARM, stroke_width=3)
        clean = axes.plot_line_graph(t, errors["clean"], add_vertex_dots=False,
                                     line_color=ACCENT, stroke_width=3)
        fast = axes.plot_line_graph(errors["t_fast"], errors["clean_fast"], add_vertex_dots=False,
                                    line_color=GOOD, stroke_width=3)
        legend = VGroup(label("MPU6050 noise, 500 Hz", 22, WARM), label("perfect IMU, 500 Hz", 22, ACCENT),
                        label("perfect IMU, 2 kHz", 22, GOOD)).arrange(DOWN, aligned_edge=LEFT)
        legend.to_corner(RIGHT + UP, buff=1.3).shift(0.2 * DOWN)
        self.play(Create(noisy), FadeIn(legend[0]), run_time=2.5)
        self.wait(1.5)
        self.play(Create(clean), FadeIn(legend[1]), run_time=2.5)
        same = label(f"peak {np.abs(errors['noisy']).max():.3f}° vs {np.abs(errors['clean']).max():.3f}°"
                     " - noise is not the culprit", 24, FG).next_to(axes, DOWN, buff=0.55)
        self.play(FadeIn(same))
        self.wait(2)
        self.play(Create(fast), FadeIn(legend[2]), run_time=2.5)
        ratio = np.abs(errors["clean"]).max() / np.abs(errors["clean_fast"]).max()
        why = tex(r"q \leftarrow q + \dot q\,\Delta t", r"\quad\Rightarrow\quad",
                  rf"\text{{error}} \propto \Delta t \;(4\times\text{{ faster}} \to {ratio:.1f}\times\text{{ smaller}})",
                  size=32).next_to(axes, DOWN, buff=0.55)
        self.play(ReplacementTransform(same, why))
        self.wait(2)
        lesson = label("Before blaming the sensor, check the integrator.", 28, WARM).to_edge(DOWN, buff=0.12)
        self.play(FadeIn(lesson))
        self.wait(4)
        self.clear_all()


class S09PositionDrift(NarratedScene):
    def construct(self):
        self.heading("Position drifts much faster")
        rows = [("error source", r"\text{velocity}", r"\text{position}", MUTED),
                ("accel white noise", r"N\sqrt t", r"N t^{3/2}/\sqrt3", FG),
                ("accel bias b", r"b\,t", r"\tfrac12 b\,t^2", FG),
                ("tilt error δθ", r"g\,\delta\theta\,t", r"\tfrac12 g\,\delta\theta\,t^2", BAD)]
        table = VGroup()
        for a, b, c, color in rows:
            table.add(VGroup(label(a, 26, color), tex(b, size=34, color=color), tex(c, size=34, color=color)))
        table.arrange(DOWN, buff=0.25, aligned_edge=LEFT).shift(1.6 * UP)
        for row in table:
            row[0].set_x(-5.0, LEFT)
            row[1].set_x(-0.5, LEFT)
            row[2].set_x(2.4, LEFT)
        for row in table:
            self.play(FadeIn(row, shift=0.1 * RIGHT), run_time=0.9)
            self.wait(0.9)
        leak = tex(r"\delta\theta = 0.1^\circ:\quad g\sin(0.1^\circ) \approx 0.017\ \text{m/s}^2", size=36)
        leak.scale(0.85).to_edge(LEFT, buff=0.7).shift(1.9 * DOWN)
        self.play(Write(leak), run_time=2)
        axes, labels = axes_box([0, 10, 2], [0, 1.0, 0.25], x_length=5.2, y_length=2.6,
                                x_label="time (s)", y_label="error (m)")
        VGroup(axes, labels).to_edge(RIGHT, buff=0.6).shift(1.8 * DOWN)
        self.play(Create(axes), FadeIn(labels), run_time=0.8)
        accel = 9.81 * np.sin(np.radians(0.1))
        curve = axes.plot(lambda x: 0.5 * accel * x ** 2, x_range=[0, 10], color=BAD, stroke_width=4)
        self.play(Create(curve), run_time=3)
        end = Dot(axes.c2p(10, 0.5 * accel * 100), color=WARM)
        tag = label(f"≈ {0.5 * accel * 100:.2f} m after 10 s", 26, WARM).next_to(end, LEFT)
        self.play(FadeIn(end), FadeIn(tag))
        self.wait(5)
        self.clear_all()


class S10Outro(NarratedScene):
    def construct(self):
        self.heading("The cure: sensor fusion")
        tilt = data.tilt_correction()
        axes, labels = axes_box([0, 30, 5], [-2, 18, 4], x_length=10, y_length=4.2,
                                x_label="time (s)", y_label="roll estimate (deg)")
        VGroup(axes, labels).shift(0.3 * DOWN)
        self.play(Create(axes), FadeIn(labels))
        only = axes.plot_line_graph(tilt["t"], tilt["only"], add_vertex_dots=False, line_color=BAD,
                                    stroke_width=3)
        fused = axes.plot_line_graph(tilt["t"], tilt["fused"], add_vertex_dots=False, line_color=GOOD,
                                     stroke_width=3)
        self.play(Create(only), run_time=2)
        tag1 = label("gyro integration (0.01 rad/s bias)", 22, BAD).next_to(axes.c2p(16, 12), UP)
        self.play(FadeIn(tag1))
        self.play(Create(fused), run_time=2)
        tag2 = label("fused with gravity (Madgwick)", 22, GOOD).next_to(axes.c2p(18, 0.5), UP)
        self.play(FadeIn(tag2))
        nxt = label("Next: Madgwick and the extended Kalman filter", 28, WARM).to_edge(DOWN, buff=0.3)
        self.play(FadeIn(nxt))
        self.wait(5)
        self.clear_all()
