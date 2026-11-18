"""Analyse and plot fly scans from `scu raster --fly --log ... --trajectory ...`.

    scu-flyplot fly1.csv fly1-traj.csv --frame-rate 50 -o fly1.png \
        --map fly1-map.png --frames-out fly1-frames.csv

--map draws where the sample went: the stage's actual path over the whole
scan and, with --frame-rate, every detector frame at its actual position,
coloured by its error along the direction of travel.

--frames-out writes one line per detector frame (assuming the detector
free-runs from each row trigger at --frame-rate): row, frame number from the
trigger, time, actual fast and slow positions, and for frames inside the grid
the nominal position and the error. This is the frame-to-position table for
data processing.

Four panels:
  1. deviation from constant speed vs position, every row (+ and - rows
     in different colours): how smooth the motion is;
  2. frame position error vs frame number (needs --frame-rate): where a
     detector free-running from each row trigger actually took each frame,
     relative to an ideal grid of the scan pitch anchored at the row's first
     grid point, measured along the direction of travel (negative = behind).
     This is the drift a speed error builds up along a row;
  3. fitted speed per row;
  4. delay from row trigger to the first grid point, per row.

A text summary is printed as well. The numbers also come from
analyse() for use in scripts. Needs matplotlib for the plot (pip install
'.[plot]'); the summary works without it.
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from dataclasses import dataclass, field


@dataclass
class Frame:
    k: int                         # frame number from the row trigger
    t: float                       # time after the trigger, s
    fast: float                    # actual fast-axis position
    grid_k: int | None = None      # frame number from the first grid point
    nominal: float | None = None   # ideal position on the scan pitch
    error: float | None = None     # actual - nominal, along travel (< 0 behind)


@dataclass
class RowStats:
    row: int
    direction: int                 # +1 / -1 in axis coordinates
    speed: float                   # fitted, over the grid region
    rms: float
    first_delay: float             # trigger -> first grid point, s
    slow: float = 0.0              # measured slow position (target if not logged)
    residual: list = field(default_factory=list)       # (pos, deviation)
    frame_error: list = field(default_factory=list)    # (frame, error um)
    frames: list = field(default_factory=list)         # Frame, all frames
    path: list = field(default_factory=list)           # (t, fast) samples


def _read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _interp(samples, t):
    """Linear interpolation of position at time t; None outside."""
    for (t0, x0), (t1, x1) in zip(samples, samples[1:]):
        if t0 <= t <= t1:
            return x0 if t1 == t0 else x0 + (x1 - x0) * (t - t0) / (t1 - t0)
    return None


def analyse(log_path: str, traj_path: str, frame_rate: float | None = None):
    """Per-row statistics from a fly log and trajectory. Returns
    (rows: list[RowStats], step: float)."""
    pts = defaultdict(list)
    for r in _read_csv(log_path):
        pts[int(r["row"])].append(r)
    traj = defaultdict(list)
    for r in _read_csv(traj_path):
        traj[int(r["row"])].append((float(r["t"]), float(r["fast_pos"])))
    step = None
    out = []
    for row in sorted(pts):
        rp = pts[row]
        targets = [float(r["fast_target"]) for r in rp]
        if len(targets) < 2 or row not in traj:
            continue
        step = abs(targets[1] - targets[0])
        d = 1 if targets[-1] > targets[0] else -1
        first, last = targets[0], targets[-1]
        t_trig = float(rp[0]["t_trigger"])
        s = sorted(traj[row])
        lo, hi = min(first, last), max(first, last)
        grid = [(t, x) for t, x in s if lo <= x <= hi]
        if len(grid) < 3:
            continue
        n = len(grid)
        mt = sum(t for t, _ in grid) / n
        mx = sum(x for _, x in grid) / n
        stt = sum((t - mt) ** 2 for t, _ in grid)
        slope = sum((t - mt) * (x - mx) for t, x in grid) / stt
        resid = [(x, x - (mx + slope * (t - mt))) for t, x in grid]
        rms = (sum(r * r for _, r in resid) / n) ** 0.5
        crosses = [r["t_cross"] for r in rp]
        first_delay = float(crosses[0]) - t_trig if crosses[0] else float("nan")
        sp = rp[0].get("slow_pos") or rp[0]["slow_target"]
        rs = RowStats(row, d, abs(slope), rms, first_delay, float(sp), resid, path=s)
        if frame_rate:
            # frame k at t_trig + k / rate; anchor the ideal grid at the
            # first frame on or after the first grid point
            period = 1.0 / frame_rate
            k0 = None
            k = 0
            while True:
                t = t_trig + k * period
                if t > s[-1][0]:
                    break
                x = _interp(s, t)
                if x is None:            # before the first sample
                    k += 1
                    continue
                fr = Frame(k, t - t_trig, x)
                if k0 is None and (x - first) * d >= 0:
                    k0, x0 = k, x
                if k0 is not None and (x - last) * d <= step / 2:
                    fr.grid_k = k - k0
                    fr.nominal = x0 + d * step * (k - k0)
                    fr.error = d * (x - fr.nominal)                    # along travel
                    rs.frame_error.append((fr.grid_k, fr.error))
                rs.frames.append(fr)
                k += 1
        out.append(rs)
    return out, step


def summary(rows, step, frame_rate=None) -> str:
    lines = []
    for d, name in ((1, "+"), (-1, "-")):
        rr = [r for r in rows if r.direction == d]
        if not rr:
            continue
        sp = [r.speed for r in rr]
        line = (f"{name} rows ({len(rr)}): speed {min(sp):.1f}..{max(sp):.1f}/s "
                f"(mean {sum(sp) / len(sp):.1f}), rms up to {max(r.rms for r in rr):.2f}, "
                f"first point {1000 * min(r.first_delay for r in rr):.1f}.."
                f"{1000 * max(r.first_delay for r in rr):.1f} ms after trigger")
        if frame_rate and step:
            target = step * frame_rate
            ends = [r.frame_error[-1] for r in rr if r.frame_error]
            if ends:
                worst = max(ends, key=lambda fe: abs(fe[1]))
                line += (f"; speed {100 * (sum(sp) / len(sp) / target - 1):+.2f}% vs "
                         f"{target:g}/s, frame drift {worst[1]:+.1f} by frame {worst[0]}")
        lines.append(line)
    return "\n".join(lines)


def plot(rows, step, frame_rate, out, title=None):
    import matplotlib
    if out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    (a1, a2), (a3, a4) = axes
    col = {1: "tab:blue", -1: "tab:orange"}
    seen = set()
    for r in rows:
        lab = None if r.direction in seen else ("+ rows" if r.direction > 0 else "- rows")
        seen.add(r.direction)
        xs, ys = zip(*r.residual)
        a1.plot(xs, ys, color=col[r.direction], alpha=0.6, lw=0.8, label=lab)
        if r.frame_error:
            fx, fy = zip(*r.frame_error)
            a2.plot(fx, fy, color=col[r.direction], alpha=0.6, lw=0.8, label=lab)
    a1.set_xlabel("fast-axis position (um)")
    a1.set_ylabel("deviation from constant speed (um)")
    a1.set_title("Motion smoothness, per row")
    a1.axhline(0, color="k", lw=0.5)
    a1.legend()
    if frame_rate:
        a2.set_title(f"Frame position error at {frame_rate:g} Hz (pitch {step:g} um)")
        a2.set_xlabel("frame number from first grid point")
        a2.set_ylabel("actual - ideal, along travel (um; < 0 behind)")
        a2.axhline(0, color="k", lw=0.5)
        a2.legend()
    else:
        a2.text(0.5, 0.5, "pass --frame-rate for frame errors", ha="center",
                va="center", transform=a2.transAxes)
        a2.set_axis_off()
    for d in (1, -1):
        rr = [r for r in rows if r.direction == d]
        if rr:
            a3.plot([r.row for r in rr], [r.speed for r in rr], "o", color=col[d],
                    label="+" if d > 0 else "-")
            a4.plot([r.row for r in rr], [1000 * r.first_delay for r in rr], "o",
                    color=col[d], label="+" if d > 0 else "-")
    if frame_rate and step:
        a3.axhline(step * frame_rate, color="k", lw=0.8, ls="--", label="target")
    a3.set_xlabel("row")
    a3.set_ylabel("speed (um/s)")
    a3.set_title("Fitted speed per row")
    a3.legend()
    a4.set_xlabel("row")
    a4.set_ylabel("ms")
    a4.set_title("Trigger to first grid point")
    a4.legend()
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    if out:
        fig.savefig(out, dpi=120)
    else:
        plt.show()


def write_frames(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["row", "frame", "t", "fast_pos", "slow_pos", "grid_frame",
                    "nominal_fast", "error_along"])
        for r in rows:
            for fr in r.frames:
                w.writerow([r.row, fr.k, f"{fr.t:.5f}", f"{fr.fast:.1f}", f"{r.slow:.1f}",
                            "" if fr.grid_k is None else fr.grid_k,
                            "" if fr.nominal is None else f"{fr.nominal:.1f}",
                            "" if fr.error is None else f"{fr.error:.2f}"])


def plot_map(rows, step, frame_rate, out, title=None):
    """Where the sample went: the stage path, and the frames coloured by
    their error along travel."""
    import matplotlib
    if out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    have_frames = bool(frame_rate) and any(r.frames for r in rows)
    fig, axes = plt.subplots(1, 2 if have_frames else 1,
                             figsize=(13 if have_frames else 7, 6.5), squeeze=False)
    ax = axes[0][0]
    xs, ys = [], []
    for r in rows:                       # rows in scan order
        if xs:                           # slow step happens with the fast axis waiting
            xs.append(xs[-1])
            ys.append(r.slow)
        for _, x in r.path:
            xs.append(x)
            ys.append(r.slow)
    ax.plot(xs, ys, color="0.35", lw=0.6)
    grid_x = [x for r in rows for x, _ in r.residual]
    if grid_x:
        x0, x1 = min(grid_x), max(grid_x)
        y0, y1 = min(r.slow for r in rows), max(r.slow for r in rows)
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                                   ls="--", color="tab:red", lw=1, label="grid"))
        ax.legend(loc="upper right")
    ax.set_title("Stage path")
    ax.set_xlabel("fast axis (um)")
    ax.set_ylabel("slow axis (um)")
    ax.set_aspect("equal", adjustable="datalim")

    if have_frames:
        ax2 = axes[0][1]
        fin = [(fr.fast, r.slow, fr.error) for r in rows for fr in r.frames
               if fr.error is not None]
        fout = [(fr.fast, r.slow) for r in rows for fr in r.frames if fr.error is None]
        if fout:
            ox, oy = zip(*fout)
            ax2.scatter(ox, oy, s=2, color="0.75", label="outside grid", rasterized=True)
        if fin:
            fx, fy, fe = zip(*fin)
            mags = sorted(abs(e) for e in fe)
            lim = max(1.0, mags[min(len(mags) - 1, int(0.995 * len(mags)))])
            clipped = mags[-1] > lim
            sc = ax2.scatter(fx, fy, c=fe, s=4, cmap="coolwarm", vmin=-lim, vmax=lim,
                             rasterized=True)
            fig.colorbar(sc, ax=ax2, extend="both" if clipped else "neither",
                         label="frame error along travel (um; < 0 behind)"
                               + (f"; max {mags[-1]:.0f}" if clipped else ""))
        ax2.set_title(f"Detector frames at {frame_rate:g} Hz (pitch {step:g} um)")
        ax2.set_xlabel("fast axis (um)")
        ax2.set_ylabel("slow axis (um)")
        ax2.set_aspect("equal", adjustable="datalim")
        if fout:
            ax2.legend(loc="upper right", markerscale=3)
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    if out:
        fig.savefig(out, dpi=130)
    else:
        plt.show()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Analyse and plot a fly scan")
    ap.add_argument("log", help="--log CSV from scu raster --fly")
    ap.add_argument("trajectory", help="--trajectory CSV from the same scan")
    ap.add_argument("--frame-rate", type=float, help="detector frame rate, Hz")
    ap.add_argument("-o", "--output", help="image file (png, pdf, svg); default: show")
    ap.add_argument("--no-plot", action="store_true", help="no panel plot")
    ap.add_argument("--map", help="also draw the sample map to this image file")
    ap.add_argument("--frames-out", help="write every detector frame's position (CSV); "
                    "needs --frame-rate")
    a = ap.parse_args(argv)
    rows, step = analyse(a.log, a.trajectory, a.frame_rate)
    if not rows:
        print("no analysable rows (need the log and trajectory of a fly scan)", file=sys.stderr)
        return 1
    print(summary(rows, step, a.frame_rate))
    if a.frames_out:
        if not a.frame_rate:
            print("--frames-out needs --frame-rate", file=sys.stderr)
            return 1
        write_frames(rows, a.frames_out)
        n = sum(len(r.frames) for r in rows)
        print(f"frames: {a.frames_out} ({n} frames)")
    if a.map:
        try:
            plot_map(rows, step, a.frame_rate, a.map, title=a.log)
        except ImportError:
            print("matplotlib not installed: pip install '.[plot]'", file=sys.stderr)
            return 2
        print(f"map: {a.map}")
    if not a.no_plot:
        try:
            plot(rows, step, a.frame_rate, a.output, title=a.log)
        except ImportError:
            print("matplotlib not installed: pip install '.[plot]' (summary above)",
                  file=sys.stderr)
            return 2
        if a.output:
            print(f"plot: {a.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
