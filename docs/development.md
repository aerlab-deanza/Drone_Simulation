# Development

## Project layout

```text
drone_sim/
  dynamics/       plant model, rotations, quaternions, frame conversions
  control/        cascaded PID, LQR
  sensors/        MPU6050 noise model, OpenCR serial reader
  estimation/     Madgwick filter, EKF
  simulation/     figure-eight pipeline, waypoint demo
  visualization/  live viewer, gimbal-lock / open-loop / statistics demos
firmware/         OpenCR Arduino sketch
notebooks/        exploratory notebooks (rendered in the docs)
tests/            pytest suite
docs/             this site (Sphinx)
videos/           Manim sources and demo-recording scripts
```

## Tests

```powershell
python -m pip install -e ".[test]"
python -m pytest
```

`pytest` runs the `tests/` suite **and** every `Example:` block in the
docstrings (doctests), so the examples in the API reference cannot go stale.

## Documentation

### Docstring style

Google style, read by `sphinx.ext.napoleon`:

```python
def rk4_step(derivative, t, y, h):
    """Advance ``y' = f(t, y)`` by one classical fourth-order Runge–Kutta step.

    Args:
        derivative: Function ``f(t, y)`` returning ``dy/dt``.
        t: Current time (s).
        y: Current state.
        h: Step size (s).

    Returns:
        State estimate at ``t + h`` with local error ``O(h⁵)``.

    Example:
        >>> ...
    """
```

Rules: every physical quantity states its **unit** and, for vectors, its
**frame** (NED/FRD or z-up/FLU). Types go in the signature (type hints), not
in the docstring.

### Building the site

```powershell
python -m pip install -e ".[docs]"
python docs/scripts/make_figures.py      # only if the code changed results
python -m sphinx -W --keep-going -b html docs docs/_build/html
start docs/_build/html/index.html
```

`-W` turns every warning (broken reference, malformed docstring) into an
error. The GitHub Actions workflow `.github/workflows/docs.yml` runs the same
build and publishes it to GitHub Pages on every push to `main`.

**One-time GitHub setup:** in the repository's *Settings → Pages*, set
*Source* to **GitHub Actions**.

### Embedding a theory video

Once a video is on YouTube, replace `TBD` with its ID in the page:

````md
```{youtube} dQw4w9WgXcQ
:title: IMU noise and drift
```
````

## Re-running the notebooks

The docs show the outputs stored in the notebooks; they are not re-executed
during the build. After changing the code:

```powershell
jupyter nbconvert --to notebook --execute --inplace notebooks/*.ipynb
```

## Re-recording the demo videos

The command recordings are rendered from real runs (the commands are
executed; their actual output is re-drawn as a PowerShell window). Real local
paths are masked. Requires `ffmpeg` on `PATH`:

```powershell
python videos/recording/record_demos.py          # install, lqr (rendered terminal)
python videos/recording/record_screen_demo.py    # static demo: real screen capture, takes over the mouse ~90 s
python videos/recording/render_replay.py         # 3D replay of the static demo
```

The live OpenCR demo needs the physical board and is recorded by hand,
following `videos/live_demo_shot_list.md`.

## Theory videos (Manim)

The four theory videos are generated with
[Manim Community](https://www.manim.community/) from `videos/manim/`, using
the project's own simulation data. Each video carries its explanation as
on-screen captions, written in `videos/captions/`. The workflow (preview,
editing captions, optional voice-over, 1080p assembly) is described in
`videos/README.md`:

```powershell
python -m pip install -e ".[videos]"
python videos/build_videos.py all v1_rotations
```
