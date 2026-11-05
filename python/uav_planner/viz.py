"""Matplotlib visualisation of terrain, no-fly zones and the planned path (optional extra)."""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

from ._core import MissionPlan, Terrain, Zones

XY = Tuple[float, float]

_MISSING = "matplotlib is required for plotting; install it with: pip install 'uav-planner[plot]'"


def plot_mission(
    terrain: Terrain,
    zones: Zones,
    plan: Optional[MissionPlan] = None,
    start: Optional[XY] = None,
    goal: Optional[XY] = None,
    path: Optional[Sequence[XY]] = None,
    title: Optional[str] = None,
    save: Optional[str] = None,
    show: bool = False,
):
    """Draw the terrain map with zones and path, plus the altitude profile below it.

    Returns the matplotlib figure. Pass ``save`` to write an image file and ``show=True``
    to open an interactive window.
    """
    try:
        import numpy as np
        from matplotlib.patches import Circle, Polygon
    except ImportError as exc:  # pragma: no cover - exercised only without matplotlib
        raise ImportError(_MISSING) from exc

    if show:
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(9, 9))
    else:
        from matplotlib.figure import Figure

        fig = Figure(figsize=(9, 9))

    has_profile = plan is not None and plan.found
    grid = fig.add_gridspec(2 if has_profile else 1, 1, height_ratios=[3, 1] if has_profile else [1])
    ax = fig.add_subplot(grid[0])

    heights = np.asarray(terrain.heights(), dtype=float)
    image = ax.imshow(
        heights,
        origin="lower",
        extent=(0.0, terrain.width_m, 0.0, terrain.height_m),
        cmap="terrain",
        aspect="equal",
    )
    fig.colorbar(image, ax=ax, label="elevation (m)", shrink=0.8)

    for x, y, r in zones.circles:
        ax.add_patch(Circle((x, y), r, facecolor="red", edgecolor="darkred", alpha=0.45))
    for pts in zones.polygons:
        ax.add_patch(Polygon(pts, closed=True, facecolor="red", edgecolor="darkred", alpha=0.45))

    line = list(path) if path is not None else (list(plan.path) if plan is not None else [])
    if line:
        ax.plot([p[0] for p in line], [p[1] for p in line], "-", color="black", linewidth=2.0, label="path")
        start = start or line[0]
        goal = goal or line[-1]
    if start is not None:
        ax.plot(*start, "o", color="lime", markeredgecolor="black", markersize=10, label="start")
    if goal is not None:
        ax.plot(*goal, "*", color="gold", markeredgecolor="black", markersize=16, label="goal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title(title or "UAV mission")
    ax.legend(loc="upper left")

    if has_profile:
        prof = plan.profile
        axp = fig.add_subplot(grid[1])
        s = [p[2] for p in prof]
        axp.fill_between(s, [p[3] for p in prof], color="saddlebrown", alpha=0.5, label="terrain")
        axp.plot(s, [p[4] for p in prof], color="navy", label="flight altitude")
        axp.set_xlabel("distance along path (m)")
        axp.set_ylabel("altitude MSL (m)")
        axp.legend(loc="upper left")

    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=150)
    if show:
        import matplotlib.pyplot as plt

        plt.show()
    return fig
