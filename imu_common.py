"""Shared bits: line parser and plot style."""
import re

# Matches the init sketch output:
#   acc g: 0.034 -0.023 1.052   gyr dps: 0.12 -0.05 0.31
# Also accepts any line with exactly 6 numbers (e.g. CSV) in the same order.
NUM = r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"
LINE_RE = re.compile(rf"acc\s*g:\s*({NUM})\s+({NUM})\s+({NUM})\s+gyr\s*dps:\s*({NUM})\s+({NUM})\s+({NUM})")
ANY_NUM = re.compile(NUM)

COLUMNS = ["t_s", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]

# Axis colours: first three slots of a CVD-checked palette (X blue, Y orange, Z aqua)
AXIS_COLORS = {"x": "#2a78d6", "y": "#eb6834", "z": "#1baf7a"}
MAG_COLOR = "#2a78d6"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"


def parse_line(line):
    """Return [ax, ay, az, gx, gy, gz] as floats, or None if the line has no data."""
    m = LINE_RE.search(line)
    if m:
        return [float(v) for v in m.groups()]
    nums = ANY_NUM.findall(line)
    if len(nums) == 6 and ("," in line or line.strip()[0:1] in "-+0123456789"):
        return [float(v) for v in nums]
    return None


def style(ax, title, ylabel):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=10, color=TEXT)
    ax.set_ylabel(ylabel, fontsize=9, color=TEXT_2)
    ax.tick_params(colors=TEXT_2, labelsize=8)
    ax.grid(True, color=GRID, linewidth=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)


# ---------------- magnetometer ----------------
# Matches the IIS2MDC sketch output:   mag mG: 312.0 -45.0 -210.0
MAG_RE = re.compile(rf"mag\s*mG:\s*({NUM})\s+({NUM})\s+({NUM})")
MAG_COLUMNS = ["t_s", "mx_mG", "my_mG", "mz_mG"]


def parse_mag_line(line):
    """Return [mx, my, mz] in mG, or None."""
    m = MAG_RE.search(line)
    return [float(v) for v in m.groups()] if m else None


def mag_heading_deg(mx, my):
    """
    Compass heading of the IIS2MDC X axis, degrees 0..360, clockwise from magnetic north.
    Valid with the chip flat and face up.
    IIS2MDC datasheet Figure 2 (top view): Z up, and Y sits 90 deg clockwise from X,
    i.e. Y points to the right of X. So heading = atan2(-Y, X).
    Works on floats or numpy arrays.
    """
    import numpy as np
    return np.degrees(np.arctan2(-np.asarray(my), np.asarray(mx))) % 360.0
