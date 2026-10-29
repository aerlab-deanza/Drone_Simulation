"""Shared pytest setup.

Force the non-interactive Agg backend before any test imports pyplot. With a GUI
backend (TkAgg), figures open real windows, and the live viewer's layout depends
on the screen size. That made the suite order-dependent.
"""

import os

os.environ["MPLBACKEND"] = "Agg"
