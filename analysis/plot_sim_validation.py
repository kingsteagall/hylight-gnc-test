"""Plot the airship-sim validation results (Exercise 2).

Input: the JSON dump written by tests/ex2-guidance.mjs (run inside the
airship-sim repo, branch ex2-guidance). Usage:
    python plot_sim_validation.py <ex2-results.json>
"""
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "ex2-results.json"
d = json.load(open(path))
wpts = np.array(d["waypoints"])

fig, axs = plt.subplots(1, 2, figsize=(13, 5.2))

ax = axs[0]
ax.plot(wpts[:, 0], wpts[:, 1], "k--", lw=1, label="mission legs")
ax.plot(wpts[:, 0], wpts[:, 1], "ks", ms=6)
for mode, color in (("baseline", "r"), ("proposed", "b")):
    t = np.array(d[mode]["traj"])
    m = d[mode]["metrics"]
    ax.plot(t[:, 1], t[:, 2], color + "-", lw=1.1,
            label=f"{mode}: rms {m['rms']:.1f} m, worst {m['worst']:.1f} m")
ax.annotate("wind + gusts", xy=(60, 65), xytext=(60, 30),
            arrowprops=dict(arrowstyle="->", color="teal", lw=2),
            color="teal", fontsize=11)
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
ax.set_title("airship-sim (full physics): same mission, same wind")
ax.legend(loc="lower right", fontsize=9); ax.axis("equal")

ax = axs[1]
for mode, color in (("baseline", "r"), ("proposed", "b")):
    t = np.array(d[mode]["traj"])
    ax.plot(t[:, 0], t[:, 5], color + "-", lw=0.9, label=mode)
ax.axhline(0, color="gray", ls=":")
ax.set_xlabel("t (s)"); ax.set_ylabel("cross-track error (m)")
ax.set_title("Cross-track error (true state)")
ax.legend(fontsize=9)

plt.tight_layout()
plt.savefig("sim_validation.png", dpi=100)
print("saved sim_validation.png")
