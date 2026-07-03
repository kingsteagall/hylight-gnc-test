"""Diagrams for the submission PDF: allocation flowchart, gyro conventions,
guidance pipeline. Pure matplotlib, no external assets."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Ellipse

INK = "#1a2733"
BLUE = "#dbeafe"; BLUE_E = "#2563eb"
ORANGE = "#ffedd5"; ORANGE_E = "#ea580c"
GREEN = "#dcfce7"; GREEN_E = "#16a34a"
GRAY = "#f1f5f9"; GRAY_E = "#64748b"


def box(ax, x, y, w, h, text, fc, ec, fs=10.5, weight="normal"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.012",
                                fc=fc, ec=ec, lw=1.6))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fs, color=INK, weight=weight, linespacing=1.45)


def arrow(ax, x0, y0, x1, y1, label=None, fs=9.5, dx=0.012):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>",
                                 mutation_scale=16, lw=1.6, color=GRAY_E))
    if label:
        ax.text((x0 + x1) / 2 + dx, (y0 + y1) / 2, label, fontsize=fs,
                color=GRAY_E, ha="left", va="center", style="italic")


# ============================================================ flowchart
fig, ax = plt.subplots(figsize=(11.5, 8.2))
ax.set_xlim(0, 1); ax.set_ylim(-0.02, 1); ax.axis("off")

box(ax, 0.30, 0.925, 0.40, 0.062,
    "Flight controller demand\n(Fx, Fz, My, Mz)   normalized, unitless",
    GRAY, GRAY_E, fs=11, weight="bold")
arrow(ax, 0.50, 0.925, 0.50, 0.878)

box(ax, 0.205, 0.775, 0.59, 0.098,
    "1 · SPLIT — wrench → one 3D force per pod (closed form)\n"
    "fx,f = fx,r = Fx      fy,f = −fy,r = Mz      "
    "fz,f = 2LrFz/L − My      fz,r = 2LfFz/L + My",
    BLUE, BLUE_E)
ax.text(0.815, 0.824, "the one free DOF:\naxial split (equal)", fontsize=9,
        color=BLUE_E, va="center", style="italic")
arrow(ax, 0.50, 0.775, 0.50, 0.728)

box(ax, 0.205, 0.635, 0.59, 0.088,
    "2 · SATURATE — if either pod needs ‖f‖ > Tmax,\n"
    "scale BOTH pods by the same factor → wrench direction preserved",
    BLUE, BLUE_E)
ax.text(0.815, 0.679, "achieved fraction\n→ lastScale( )\n(anti-windup)",
        fontsize=9, color=BLUE_E, va="center", style="italic")
arrow(ax, 0.50, 0.635, 0.50, 0.588)

box(ax, 0.205, 0.495, 0.59, 0.088,
    "3 · INVERT (per pod) — force vector → actuator angles\n"
    "T = ‖f‖        β = asin( fy / T )        α = atan2( −fx , −fz )",
    BLUE, BLUE_E)
ax.text(0.815, 0.539, "exact, no iteration", fontsize=9, color=BLUE_E,
        va="center", style="italic")
arrow(ax, 0.50, 0.495, 0.50, 0.448, label="  raw targets (may jump)")

# continuity container
ax.add_patch(FancyBboxPatch((0.115, 0.118), 0.77, 0.325,
                            boxstyle="round,pad=0.012", fc="#fff7ed",
                            ec=ORANGE_E, lw=2))
ax.text(0.5, 0.415, "4 · CONTINUITY — stateful; servos never asked to jump",
        ha="center", fontsize=11.5, weight="bold", color=INK)

steps = [
    ("deadband\n+ hysteresis\n\nT < 0.1% Tmax ⇒\nangles freeze\n(don't chase noise)"),
    ("singularity\nhold\n\nβ = ±90° ⇒\nα undefined,\nhold previous"),
    ("shortest-path\nunwrap of α\n\nno ±180° flips;\nrange stops\nif configured"),
    ("slew-rate\nlimit\n\nα, β ≤ 120°/s\nT ≤ 2 Tmax/s\n(real servo speed)"),
    ("thrust easing\nduring swings\n\nT × cos(err):\nfade while far,\nrestore on arrival"),
]
w = 0.136
for i, s in enumerate(steps):
    x = 0.135 + i * (w + 0.0145)
    box(ax, x, 0.150, w, 0.235, s, ORANGE, ORANGE_E, fs=8.6)
    if i < len(steps) - 1:
        arrow(ax, x + w, 0.268, x + w + 0.0145, 0.268)

arrow(ax, 0.50, 0.118, 0.50, 0.072)
box(ax, 0.28, 0.005, 0.44, 0.062,
    "Actuator commands\n(T_f, α_f, β_f)   (T_r, α_r, β_r)",
    GREEN, GREEN_E, fs=11, weight="bold")

plt.tight_layout()
plt.savefig("flowchart_alloc.png", dpi=150, bbox_inches="tight")
plt.close(fig)

# ============================================================ conventions
fig, axs = plt.subplots(1, 2, figsize=(11.5, 3.6))
for ax in axs:
    ax.set_xlim(0, 10); ax.set_ylim(0, 5); ax.axis("off"); ax.set_aspect("equal")

ax = axs[0]
ax.set_title("Side view — Nose·Right·Down body frame", fontsize=11, color=INK)
ax.add_patch(Ellipse((5, 2.6), 6.4, 1.7, fc="#e2e8f0", ec=GRAY_E, lw=1.5))
ax.annotate("", xy=(9.4, 2.6), xytext=(5, 2.6),
            arrowprops=dict(arrowstyle="-|>", color=BLUE_E, lw=2))
ax.text(9.45, 2.6, "x (nose)", fontsize=10, color=BLUE_E, va="center")
ax.annotate("", xy=(5, 0.4), xytext=(5, 2.6),
            arrowprops=dict(arrowstyle="-|>", color=BLUE_E, lw=2))
ax.text(5.12, 0.5, "z (down)", fontsize=10, color=BLUE_E)
ax.plot(5, 2.6, "ko", ms=5); ax.text(5.14, 2.78, "G", fontsize=10, color=INK)
for x0, lab in ((8.2, "front gyro (+Lf)"), (1.8, "rear gyro (−Lr)")):
    ax.annotate("", xy=(x0, 4.5), xytext=(x0, 3.1),
                arrowprops=dict(arrowstyle="-|>", color="#dc2626", lw=2.4))
    ax.text(x0, 1.35, lab, fontsize=9, color=INK, ha="center")
ax.text(1.8, 4.65, "T (neutral = up)", fontsize=8.6, color="#dc2626", ha="center")
ax.annotate("α > 0 tilts the thrust back", xy=(8.05, 4.35), xytext=(4.6, 4.75),
            fontsize=9, color=ORANGE_E, ha="center",
            arrowprops=dict(arrowstyle="->", color=ORANGE_E,
                            connectionstyle="arc3,rad=0.3"))

ax = axs[1]
ax.set_title("Top view — β tilts the thrust sideways", fontsize=11, color=INK)
ax.add_patch(Ellipse((5, 2.5), 6.4, 1.6, fc="#e2e8f0", ec=GRAY_E, lw=1.5))
ax.annotate("", xy=(9.4, 2.5), xytext=(5, 2.5),
            arrowprops=dict(arrowstyle="-|>", color=BLUE_E, lw=2))
ax.text(9.45, 2.5, "x", fontsize=10, color=BLUE_E, va="center")
ax.annotate("", xy=(5, 0.5), xytext=(5, 2.5),
            arrowprops=dict(arrowstyle="-|>", color=BLUE_E, lw=2))
ax.text(5.12, 0.6, "y (right)", fontsize=10, color=BLUE_E)
for x0 in (8.2, 1.8):
    ax.plot(x0, 2.5, "o", ms=9, mfc="#fecaca", mec="#dc2626")
ax.annotate("β > 0 → thrust to the RIGHT", xy=(8.2, 2.15), xytext=(1.0, 4.4),
            fontsize=9, color=ORANGE_E,
            arrowprops=dict(arrowstyle="->", color=ORANGE_E,
                            connectionstyle="arc3,rad=-0.25"))
ax.text(0.4, 0.35, "t(α,β) = (−sinα·cosβ,  sinβ,  −cosα·cosβ)",
        fontsize=10.5, color=INK, family="monospace")

plt.tight_layout()
plt.savefig("conventions.png", dpi=150, bbox_inches="tight")
plt.close(fig)

# ============================================================ guidance flow
fig, ax = plt.subplots(figsize=(11.5, 3.1))
ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")

chain = [
    ("GPS\nposition + velocity", GRAY, GRAY_E),
    ("project onto LEG\ncross-track e,\nalong-track s", BLUE, BLUE_E),
    ("course_cmd =\nleg azimuth +\nclamp(−atan(e/L1), ±40°)", BLUE, BLUE_E),
    ("+ crab\nslow integral of\n(course_cmd − course_meas)", ORANGE, ORANGE_E),
    ("yaw_sp\n(unwrapped,\nshortest-arc PID)", GREEN, GREEN_E),
]
w = 0.165
for i, (t, fc, ec) in enumerate(chain):
    x = 0.02 + i * (w + 0.032)
    box(ax, x, 0.42, w, 0.40, t, fc, ec, fs=9.3)
    if i < len(chain) - 1:
        arrow(ax, x + w, 0.62, x + w + 0.032, 0.62)

box(ax, 0.235, 0.04, 0.36, 0.22,
    "switch leg when the perpendicular plane is crossed\n"
    "(anticipated by the turn radius R = V/r_max) — no bearing thrash",
    GRAY, GRAY_E, fs=9)
ax.text(0.66, 0.15,
        "wind changes? the integral finds the new crab.\n"
        "crab saturated? slow down — don't leave the line.",
        fontsize=9.3, color=ORANGE_E, style="italic", va="center")

plt.tight_layout()
plt.savefig("flowchart_guidance.png", dpi=150, bbox_inches="tight")
plt.close(fig)

print("saved flowchart_alloc.png, conventions.png, flowchart_guidance.png")
