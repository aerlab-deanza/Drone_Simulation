"""Central limit theorem demo: why sensor noise is modelled as Gaussian.

The mean of many independent errors tends toward a Gaussian whatever their
individual distribution. Sensor noise is the sum of many small independent
effects (thermal noise, quantization, vibration), which is why the IMU model
and the Kalman filter both assume Gaussian noise.

Run ``python -m drone_sim.visualization.statistics_demo`` to save
``outputs/sim01_clt.png``.
"""

from __future__ import annotations

import numpy as np


def sample_means(distribution: str, sample_size: int, n_trials: int = 5000,
                 rng: np.random.Generator | None = None) -> np.ndarray:
    """Return ``n_trials`` means of ``sample_size`` draws from a distribution.

    Args:
        distribution: ``"uniform"`` (on [0, 1]), ``"exponential"`` (λ = 1) or
            ``"laplace"`` (0, 1).
        sample_size: Draws averaged per trial (``N``).
        n_trials: Number of means to return.
        rng: Random generator; a seeded default is used if None.

    Returns:
        Array of shape ``(n_trials,)``.

    Example:
        >>> means = sample_means("exponential", 100, rng=np.random.default_rng(0))
        >>> round(float(means.std()), 2)  # σ/√N = 1/10
        0.1
    """
    rng = np.random.default_rng(42) if rng is None else rng
    draws = {
        "uniform": lambda: rng.uniform(0, 1, (n_trials, sample_size)),
        "exponential": lambda: rng.exponential(1, (n_trials, sample_size)),
        "laplace": lambda: rng.laplace(0, 1, (n_trials, sample_size)),
    }
    return draws[distribution]().mean(axis=1)


def plot_central_limit_theorem(sample_sizes: tuple[int, ...] = (1, 5, 30, 100)):
    """Plot histograms of sample means with a fitted normal curve.

    Returns:
        matplotlib.figure.Figure: A grid with one row per distribution and one
        column per sample size.
    """
    import matplotlib.pyplot as plt
    from scipy import stats

    rng = np.random.default_rng(42)
    names = {"uniform": "Uniform [0,1]", "exponential": "Exponential(λ=1)",
             "laplace": "Laplace(0,1)"}
    fig, axes = plt.subplots(len(names), len(sample_sizes), figsize=(14, 9))
    fig.suptitle("Central Limit Theorem\nSample means converge to Gaussian as N grows",
                 fontsize=13)
    for row, (key, label) in enumerate(names.items()):
        for col, n in enumerate(sample_sizes):
            means = sample_means(key, n, rng=rng)
            ax = axes[row, col]
            ax.hist(means, bins=60, density=True, color="steelblue", alpha=0.7,
                    edgecolor="none")
            mu, sigma = means.mean(), means.std()
            grid = np.linspace(mu - 4*sigma, mu + 4*sigma, 200)
            ax.plot(grid, stats.norm.pdf(grid, mu, sigma), "r-", lw=2)
            ax.set_title(f"N={n}", fontsize=10)
            if col == 0:
                ax.set_ylabel(label, fontsize=9)
            ax.set_xlabel("Sample mean", fontsize=8)
            ax.tick_params(labelsize=7)
    fig.tight_layout()
    return fig


if __name__ == "__main__":
    plot_central_limit_theorem().savefig("outputs/sim01_clt.png", dpi=110)
    print("Saved outputs/sim01_clt.png")
