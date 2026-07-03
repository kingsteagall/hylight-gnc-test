"""Extract and analyze the Annex B flight log (Plotly HTML with binary buffers).

Usage: python extract_log.py <mission_hylight.html>
Produces flight_overview.png, flight_zoom.png and prints per-phase statistics.
"""
import base64
import json
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_traces(path):
    html = open(path, encoding="utf-8").read()
    m = re.search(r'Plotly\.newPlot\(\s*"([^"]+)"\s*,\s*(\[.*)', html, re.DOTALL)
    data, _ = json.JSONDecoder().raw_decode(m.group(2))

    def arr(v):
        if isinstance(v, dict) and "bdata" in v:
            return np.frombuffer(base64.b64decode(v["bdata"]), dtype=np.dtype(v["dtype"]))
        return np.asarray(v)

    return {t["name"]: (arr(t["x"]), arr(t["y"])) for t in data}


def angerr(a, b):
    return (a - b + 180) % 360 - 180


def main(path):
    tr = load_traces(path)

    # Mode timeline
    mx, my = tr["Flight mode"]
    segs, cur, t0 = [], my[0], mx[0]
    for i in range(1, len(my)):
        if my[i] != cur:
            segs.append((cur, t0, mx[i]))
            cur, t0 = my[i], mx[i]
    segs.append((cur, t0, mx[-1]))
    print("Mode timeline:")
    for mo, a, b in segs:
        print(f"  {mo:10s} {a:7.1f} -> {b:7.1f}  ({b - a:6.1f}s)")

    x, y = tr["Yaw"]
    xs, ys = tr["Yaw setpoint"]
    # Hold-interpolate the setpoint (linear interp across a +-180 wrap flip
    # manufactures spurious ~180 deg errors) and mask the 243-250 s flip burst.
    idx = np.clip(np.searchsorted(xs, x, side="right") - 1, 0, len(ys) - 1)
    err = angerr(y, ys[idx])

    sl = (x >= 130) & (x <= 260) & ~((x >= 242) & (x <= 250))
    e = err[sl]
    print(f"cruise 130-260s (flips masked): yaw_err rms={np.sqrt(np.mean(e**2)):.1f} "
          f"max={np.abs(e).max():.1f} deg")
    ed = e - e.mean()
    f = np.fft.rfftfreq(len(ed), np.median(np.diff(x[sl])))
    P = np.abs(np.fft.rfft(ed * np.hanning(len(ed)))) ** 2
    print(f"dominant yaw-hunt period ~{1 / f[np.argmax(P[1:]) + 1]:.0f} s")

    s5 = (x >= 85) & (x <= 130)
    unw = np.degrees(np.unwrap(np.radians(y[s5])))
    print(f"turnaround 85-130s: net {unw[-1] - unw[0]:+.0f} deg in {x[s5][-1] - x[s5][0]:.0f} s")

    x2, y2 = tr["Vx"]
    xs2, ys2 = tr["Vx setpoint"]
    sp2 = np.interp(x2, xs2, ys2)
    c = (x2 >= 130) & (x2 <= 260) & (sp2 > 1)
    print(f"cruise vx: sp={sp2[c].mean():.2f} act={y2[c].mean():.2f} m/s")
    pos = (x2 >= 263.5) & (x2 <= 351.5)
    print(f"position mode: min vx={y2[pos].min():.2f} m/s (pushed backwards)")
    print(f"yaw-sp wrap flips (>300 deg jumps): {np.sum(np.abs(np.diff(ys)) > 300)}")

    x3, y3 = tr["Altitude"]
    xs3, ys3 = tr["Altitude setpoint"]
    ae = y3 - np.interp(x3, xs3, ys3)
    sm = (x3 >= 40) & (x3 <= 260)
    xp, yp = tr["Pitch"]
    xps, yps = tr["Pitch setpoint"]
    pe = yp - np.interp(xp, xps, yps)
    sp_ = (xp >= 40) & (xp <= 260)
    print(f"alt err rms={np.sqrt(np.mean(ae[sm]**2)):.2f} max={np.abs(ae[sm]).max():.2f} m; "
          f"pitch err rms={np.sqrt(np.mean(pe[sp_]**2)):.2f} max={np.abs(pe[sp_]).max():.2f} deg")

    # Overview plot
    fig, axs = plt.subplots(4, 1, figsize=(16, 14), sharex=True)
    axs[0].plot(x, y, "b-", lw=0.7, label="Yaw")
    axs[0].plot(xs, ys, "r-", lw=0.7, label="Yaw sp")
    axs[0].set_ylabel("Yaw (deg)"); axs[0].legend()
    axs[1].plot(x, err, "k-", lw=0.7); axs[1].set_ylabel("Yaw err (deg)")
    axs[2].plot(x2, y2, "b-", lw=0.7, label="Vx")
    axs[2].plot(xs2, ys2, "r-", lw=0.7, label="Vx sp")
    axs[2].set_ylabel("Vx (m/s)"); axs[2].legend()
    x3, y3 = tr["Altitude"]; xs3, ys3 = tr["Altitude setpoint"]
    axs[3].plot(x3, y3, "b-", lw=0.7, label="Alt")
    axs[3].plot(xs3, ys3, "r-", lw=0.7, label="Alt sp")
    axs[3].set_ylabel("Alt (m)"); axs[3].set_xlabel("t (s)"); axs[3].legend()
    for ax in axs:
        ax.axvspan(33.4, 263.5, alpha=0.06, color="orange")
        ax.axvspan(263.5, 351.5, alpha=0.06, color="cyan")
    plt.tight_layout(); plt.savefig("flight_overview.png", dpi=90)

    # Zoom on the turnaround
    fig, axs = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    sl = (x >= 33) & (x <= 150); sls = (xs >= 33) & (xs <= 150)
    axs[0].plot(x[sl], y[sl], "b-", lw=0.9, label="Yaw")
    axs[0].plot(xs[sls], ys[sls], "r-", lw=0.9, label="Yaw sp")
    axs[0].legend(); axs[0].set_ylabel("Yaw")
    axs[1].plot(x[sl], err[sl], "k-", lw=0.9); axs[1].set_ylabel("Yaw err")
    s2 = (x2 >= 33) & (x2 <= 150); s2s = (xs2 >= 33) & (xs2 <= 150)
    axs[2].plot(x2[s2], y2[s2], "b-", lw=0.9, label="Vx")
    axs[2].plot(xs2[s2s], ys2[s2s], "r-", lw=0.9, label="Vx sp")
    axs[2].legend(); axs[2].set_ylabel("Vx"); axs[2].set_xlabel("t (s)")
    plt.tight_layout(); plt.savefig("flight_zoom.png", dpi=90)
    print("saved flight_overview.png, flight_zoom.png")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "mission_hylight.html")
