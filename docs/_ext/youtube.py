"""Tiny Sphinx directive for embedding YouTube videos.

Usage in MyST::

    ```{youtube} dQw4w9WgXcQ
    :title: Gimbal lock and quaternions
    ```

Until a video is uploaded, pass ``TBD`` as the ID. A clearly marked
placeholder is rendered instead of an iframe, so the strict build still
passes.
"""

from __future__ import annotations

import hashlib
import html
from pathlib import Path

from docutils import nodes
from docutils.parsers.rst import Directive, directives


class YouTube(Directive):
    """Embed a responsive 16:9 YouTube iframe (or a placeholder for ``TBD``)."""

    required_arguments = 1
    option_spec = {"title": directives.unchanged}

    def run(self) -> list[nodes.Node]:
        video_id = self.arguments[0].strip()
        title = html.escape(self.options.get("title", "Video"))
        if video_id.upper() == "TBD":
            markup = (f'<div class="video-placeholder"><strong>{title}</strong>'
                      "<br>Video coming soon.</div>")
        else:
            safe_id = html.escape(video_id)
            markup = (
                '<div class="video-embed">'
                f'<iframe src="https://www.youtube-nocookie.com/embed/{safe_id}" '
                f'title="{title}" loading="lazy" allowfullscreen '
                'allow="accelerometer; encrypted-media; gyroscope; picture-in-picture">'
                "</iframe></div>"
            )
        return [nodes.raw("", markup, format="html")]


class LocalVideo(Directive):
    """Embed an MP4 from ``docs/_static/videos`` with a native player.

    Usage in MyST (the path is relative to ``_static/videos``)::

        ```{localvideo} theory/v1_rotations.mp4
        :title: Euler angles, gimbal lock, and quaternions
        ```

    The ``src`` is made relative to the current page, so it works at any
    folder depth and on GitHub Pages.
    """

    required_arguments = 1
    option_spec = {"title": directives.unchanged}

    def run(self) -> list[nodes.Node]:
        env = self.state.document.settings.env
        depth = env.docname.count("/")
        prefix = "../" * depth
        relative = self.arguments[0].strip().lstrip("/")
        path = html.escape(relative)
        title = html.escape(self.options.get("title", "Video"))
        version = _fingerprint(Path(env.srcdir) / "_static" / "videos" / relative)
        markup = (
            f'<video class="demo-video" controls playsinline preload="metadata" title="{title}" '
            f'src="{prefix}_static/videos/{path}?v={version}"></video>'
        )
        return [nodes.raw("", markup, format="html")]


def _fingerprint(path: Path) -> str:
    """Short content hash, appended to the URL so browsers never show a stale video."""
    if not path.exists():
        return "missing"
    return hashlib.sha1(path.read_bytes()).hexdigest()[:10]


def setup(app) -> dict[str, object]:
    app.add_directive("youtube", YouTube)
    app.add_directive("localvideo", LocalVideo)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
