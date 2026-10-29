"""Shared style, timing and helpers for the theory videos.

Every scene derives from :class:`NarratedScene` (2D) or
:class:`NarratedThreeDScene` (3D). Their ``play`` and ``wait`` calls are
multiplied by a per-scene *time scale*, so a scene can be stretched or
compressed to fit the narrator's recorded audio without editing the
animation code. ``build_videos.py`` computes the scales and writes them to
``videos/build/timings.json``; without that file every scale is 1.0.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
from manim import (
    DOWN,
    LEFT,
    RIGHT,
    UL,
    UP,
    Create,
    FadeIn,
    FadeOut,
    MathTex,
    Rectangle,
    Scene,
    Tex,
    Text,
    ThreeDScene,
    VGroup,
    Wait,
    Write,
    config,
)

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))  # make ``drone_sim`` importable for project data

TIMINGS_FILE = REPO / "videos" / "build" / "timings.json"

# -- palette ----------------------------------------------------------------
BG = "#0f1117"
FG = "#e8e8e8"
MUTED = "#8a8f98"
ACCENT = "#4fa3ff"      # blue: truth / main quantity
WARM = "#ffb347"        # orange: estimates
GOOD = "#5fd38d"        # green: correct / converged
BAD = "#ff6b6b"         # red: errors / singularities
PURPLE = "#c792ea"
X_COLOR, Y_COLOR, Z_COLOR = "#ff6b6b", "#5fd38d", "#4fa3ff"

config.background_color = BG
Text.set_default(font="Segoe UI")

# Coloured symbols reused across videos.
TEX_COLORS = {r"\phi": X_COLOR, r"\theta": Y_COLOR, r"\psi": Z_COLOR}


STATS_FILE = REPO / "videos" / "build" / "scene_stats.json"


def load_time_scale(scene_name: str) -> tuple[float, float]:
    """Return ``(play_scale, wait_scale)`` for ``scene_name`` (``"<module>.<Class>"``).

    Animations are stretched only modestly; most of the extra narration time
    goes into the pauses, so equations never write themselves in slow motion.
    """
    override = os.environ.get("NARRATION_TIME_SCALE")
    if override:
        return float(override), float(override)
    if TIMINGS_FILE.exists():
        entry = json.loads(TIMINGS_FILE.read_text(encoding="utf-8")).get(scene_name)
        if isinstance(entry, dict):
            return float(entry["play"]), float(entry["wait"])
        if entry is not None:
            return float(entry), float(entry)
    return 1.0, 1.0


class NarrationTiming:
    """Mixin that scales ``play`` and ``wait`` durations to fit the narration.

    It also records the unscaled totals of animation and waiting time, which
    ``build_videos.py`` uses to split the stretch between the two.
    """

    play_scale: float = 1.0
    wait_scale: float = 1.0

    def setup(self) -> None:  # noqa: D401 - Manim hook
        module = type(self).__module__.split(".")[-1]
        self.scene_key = f"{module}.{type(self).__name__}"
        self.play_scale, self.wait_scale = load_time_scale(self.scene_key)
        self._play_total = 0.0
        self._wait_total = 0.0
        super().setup()

    def play(self, *animations, run_time: float | None = None, **kwargs):
        # Manim implements wait() as play(Wait(...)); that time was already
        # scaled and counted in our wait(), so pass it straight through.
        if animations and all(isinstance(a, Wait) for a in animations):
            if run_time is not None:
                kwargs["run_time"] = run_time
            return super().play(*animations, **kwargs)
        if run_time is None:
            run_time = max([getattr(a, "run_time", 1.0) for a in animations] or [1.0])
        self._play_total += run_time
        return super().play(*animations, run_time=run_time * self.play_scale, **kwargs)

    def wait(self, duration: float = 1.0, **kwargs):
        self._wait_total += duration
        return super().wait(duration * self.wait_scale, **kwargs)

    def tear_down(self) -> None:  # noqa: D401 - Manim hook
        stats = json.loads(STATS_FILE.read_text(encoding="utf-8")) if STATS_FILE.exists() else {}
        stats[self.scene_key] = {"play": round(self._play_total, 4), "wait": round(self._wait_total, 4)}
        STATS_FILE.parent.mkdir(parents=True, exist_ok=True)
        STATS_FILE.write_text(json.dumps(stats, indent=2, sort_keys=True), encoding="utf-8")
        super().tear_down()

    # -- helpers shared by every scene -----------------------------------------
    def title_card(self, title: str, subtitle: str | None = None, hold: float = 1.5) -> None:
        """Full-screen title, then fade out."""
        heading = Text(title, font_size=52, color=FG, weight="BOLD")
        group = VGroup(heading)
        if subtitle:
            group.add(Text(subtitle, font_size=28, color=MUTED))
        group.arrange(DOWN, buff=0.35)
        self.play(FadeIn(group, shift=0.2 * UP), run_time=1.0)
        self.wait(hold)
        self.play(FadeOut(group), run_time=0.6)

    def heading(self, text: str) -> Text:
        """Small section heading pinned to the top-left corner."""
        label = Text(text, font_size=30, color=MUTED).to_corner(UL, buff=0.4)
        self.play(FadeIn(label), run_time=0.5)
        return label

    def code_ref(self, reference: str) -> VGroup:
        """Show a 'where in the code' tag at the bottom of the frame."""
        text = Text(reference, font="Consolas", font_size=20, color=ACCENT)
        box = Rectangle(width=text.width + 0.4, height=text.height + 0.25,
                        stroke_color=ACCENT, stroke_width=1.5)
        tag = VGroup(box, text).to_edge(DOWN, buff=0.3)
        self.play(FadeIn(tag, shift=0.1 * UP), run_time=0.5)
        return tag

    def clear_all(self, run_time: float = 0.6) -> None:
        """Fade out every mobject on screen."""
        if self.mobjects:
            self.play(*[FadeOut(m) for m in self.mobjects], run_time=run_time)


class NarratedScene(NarrationTiming, Scene):
    """2D scene with narration-aware timing."""


class NarratedThreeDScene(NarrationTiming, ThreeDScene):
    """3D scene with narration-aware timing."""


def tex(*parts: str, size: float = 40, color: str = FG, **kwargs) -> MathTex:
    """MathTex in the house style. Pass several parts to colour them separately."""
    return MathTex(*parts, font_size=size, color=color, **kwargs)


def label(text: str, size: float = 26, color: str = FG, **kwargs) -> Text:
    """Plain text label."""
    return Text(text, font_size=size, color=color, **kwargs)


def bullet_list(items: list[str], size: float = 28, buff: float = 0.32) -> VGroup:
    """Left-aligned bullet list of plain-text items."""
    rows = VGroup(*[Text("•  " + item, font_size=size, color=FG) for item in items])
    rows.arrange(DOWN, aligned_edge=LEFT, buff=buff)
    return rows


def finding_badge(number: int, text: str) -> VGroup:
    """Rounded 'Finding N' badge used when a scene presents a project finding."""
    from manim import RoundedRectangle

    words = Text(f"Finding {number}  ·  {text}", font_size=24, color=BG, weight="BOLD")
    box = RoundedRectangle(corner_radius=0.15, width=words.width + 0.5, height=words.height + 0.3,
                           fill_color=WARM, fill_opacity=1, stroke_width=0)
    return VGroup(box, words)


__all__ = [
    "ACCENT", "BAD", "BG", "FG", "GOOD", "MUTED", "PURPLE", "WARM",
    "X_COLOR", "Y_COLOR", "Z_COLOR", "NarratedScene", "NarratedThreeDScene",
    "bullet_list", "finding_badge", "label", "tex", "np", "Create", "Write", "Tex", "RIGHT",
]
