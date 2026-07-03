"""Exercise 2 Q2 demonstration: bearing-chasing vs course-based guidance.

A deliberately minimal planar airship model that reproduces the pathologies
visible in the Annex B log, then shows the proposed guidance fixing them.
Both runs share the SAME vehicle, wind, and yaw/speed inner loops — only the
guidance layer differs.

Vehicle model (the parts that matter for lateral drift, nothing else):
  - heading psi with slow yaw dynamics (rate limit ~6 deg/s as measured in
    the log's 261 deg / 45 s turnaround; first-order rate response)
  - a yaw disturbance torque from gusts + slender-body (Munk-like) moment
  - airspeed v along the nose, first-order response, capped at 3 m/s
  - ground velocity = airspeed * heading + wind(t): heading != course
  - wind: steady crosswind + slow sinusoidal gusts + colored noise
    (the log shows ~30 s hunting and -2.4 m/s pushback => wind O(airspeed))

BASELINE guidance (what the log behaves like):
  - yaw_sp = bearing(position -> next waypoint), recomputed every tick,
    reported wrapped to (-180, 180] (the +-180 flips of the log appear in
    this signal whenever the bearing sits near south)
  - the yaw inner loop itself is given the benefit of the doubt (shortest-arc
    error): the point is that EVEN WITH a sane yaw PID, bearing-chasing
    cannot hold the line — heading tracks the bearing, wind adds the drift
  - waypoint switch on an acceptance radius only -> bearing swings wildly
    when passing beside a waypoint

PROPOSED guidance (the answer to Q2):
  - track the LEG (segment between waypoints): cross-track error e
  - course_cmd = leg_azimuth + bounded correction (L1-style, clamp +-40 deg)
  - crab found by slow integral action on (course_cmd - course_measured):
    yaw_sp = course_cmd + crab  -> steady wind = steady crab, no wind sensor
  - every angular error taken as shortest arc; setpoint continuous (unwrap)
  - waypoint switch by crossing the perpendicular plane, anticipated by the
    physical turn radius (R = V/r_max ~ 29 m, exactly what PX4's L1 does),
    with the acceptance radius as fallback -> no bearing thrash, no wide
    ballooning exits at corners

Usage: python guidance_demo.py   (writes guidance_demo.png + prints metrics)
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

D2R = np.pi / 180.0


def wrap(a):
    """Shortest-arc wrap to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


# ----------------------------------------------------------------------------
# Shared plant and inner loops
# ----------------------------------------------------------------------------
DT = 0.1          # [s]
V_CRUISE = 3.0    # [m/s] airspeed setpoint
V_TAU = 3.0       # [s] airspeed response
R_MAX = 6.0 * D2R # [rad/s] yaw-rate authority (log: 261 deg in 45 s)
R_TAU = 1.5       # [s] yaw-rate response
KP_YAW = 1.2      # yaw PID (rate demand per rad of error)

WIND_BASE = np.array([0.0, 1.2])  # [m/s] steady crosswind (+y) for +x legs
GUST_AMP = 0.6                    # [m/s] slow sinusoidal gust
GUST_T = 30.0                     # [s] gust period (log: ~30 s hunting)
MUNK_GAIN = 0.35                  # yaw disturbance from lateral relative wind


class Plant:
    """Planar airship: slow yaw, slow airspeed, wind makes course != heading."""

    def __init__(self, seed=7):
        self.pos = np.zeros(2)
        self.psi = 0.0
        self.r = 0.0
        self.v = 0.0
        self.rng = np.random.default_rng(seed)
        self.noise = 0.0
        self.t = 0.0

    def wind(self):
        g = GUST_AMP * np.sin(2.0 * np.pi * self.t / GUST_T)
        return WIND_BASE + np.array([0.0, g])

    def step(self, yaw_err, v_sp):
        # colored yaw disturbance: gusty Munk-like moment on the bare hull
        self.noise += (-self.noise / 8.0 + 0.02 * self.rng.standard_normal()) * DT
        w = self.wind()
        heading = np.array([np.cos(self.psi), np.sin(self.psi)])
        lateral_wind = -w[0] * np.sin(self.psi) + w[1] * np.cos(self.psi)
        disturb = MUNK_GAIN * D2R * lateral_wind + self.noise

        r_cmd = np.clip(KP_YAW * yaw_err, -R_MAX, R_MAX)
        self.r += ((r_cmd - self.r) / R_TAU + disturb) * DT
        self.r = np.clip(self.r, -R_MAX, R_MAX)
        self.psi = wrap(self.psi + self.r * DT)

        self.v += (min(v_sp, V_CRUISE) - self.v) / V_TAU * DT
        self.pos = self.pos + (self.v * heading + w) * DT
        self.t += DT
        vg = self.v * heading + w
        return vg


# Out-and-back mission with long legs: straight-line tracking is the ask,
# and legs must be long relative to the physical turn radius (~29 m at
# 6 deg/s) so the corner arc does not dominate the comparison.
WAYPOINTS = np.array([[0.0, 0.0], [350.0, 0.0], [350.0, 130.0],
                      [0.0, 130.0], [0.0, 0.0]])
ACCEPT_R = 6.0
T_END = 900.0


def run(guidance):
    p = Plant()
    wp_i = 1
    crab = 0.0
    sp_prev = 0.0
    log = {"pos": [], "e": [], "sp": [], "psi": [], "t": [], "leg": []}
    flips = 0

    while p.t < T_END and wp_i < len(WAYPOINTS):
        a, b = WAYPOINTS[wp_i - 1], WAYPOINTS[wp_i]
        leg = b - a
        leg_len = np.linalg.norm(leg)
        u = leg / leg_len
        rel = p.pos - a
        along = rel @ u
        e = -rel[0] * u[1] + rel[1] * u[0]  # signed cross-track (+left of leg)
        dist_wp = np.linalg.norm(b - p.pos)

        if guidance == "baseline":
            # bearing to the waypoint, recomputed from the drifting position;
            # the inner yaw loop gets the benefit of the doubt (shortest arc)
            sp = np.arctan2(b[1] - p.pos[1], b[0] - p.pos[0])
            yaw_err = wrap(sp - p.psi)
            switch = dist_wp < ACCEPT_R
        else:
            leg_az = np.arctan2(u[1], u[0])
            course_cmd = leg_az + np.clip(-np.arctan2(e, 25.0), -40 * D2R, 40 * D2R)
            vg = p.v * np.array([np.cos(p.psi), np.sin(p.psi)]) + p.wind()
            if np.linalg.norm(vg) > 0.5:      # course measurable
                course_meas = np.arctan2(vg[1], vg[0])
                crab += 0.35 * wrap(course_cmd - course_meas) * DT
                crab = np.clip(crab, -35 * D2R, 35 * D2R)
            # setpoint kept CONTINUOUS (unwrapped) for the yaw PID — the
            # ±180° flip pathology of the flight log cannot happen here
            sp = sp_prev + wrap(course_cmd + crab - sp_prev)
            yaw_err = wrap(sp - p.psi)        # shortest arc, always
            # switch anticipated by the physical turn radius (PX4-L1 style)
            r_turn = V_CRUISE / R_MAX
            switch = (along >= leg_len - r_turn) or (dist_wp < ACCEPT_R)

        if abs(sp - sp_prev) > 170 * D2R:
            flips += 1
        sp_prev = sp

        p.step(yaw_err, V_CRUISE)
        log["pos"].append(p.pos.copy())
        log["e"].append(e)
        log["sp"].append(sp)
        log["psi"].append(p.psi)
        log["t"].append(p.t)
        log["leg"].append(wp_i)
        if switch:
            wp_i += 1

    for k in log:
        log[k] = np.asarray(log[k])
    log["flips"] = flips
    log["done"] = wp_i >= len(WAYPOINTS)
    return log


def metrics(lg):
    # WHOLE-leg cross-track, no exclusions: the proposed guidance pays for its
    # own corner-anticipation transients (an earlier settled-portion filter
    # turned out to trim samples only from the proposed run — the baseline
    # switches inside the acceptance radius and never had anything excluded).
    stats = []
    for leg in np.unique(lg["leg"]):
        m = lg["leg"] == leg
        if m.sum() < 20:
            continue
        e = lg["e"][m]
        stats.append((np.sqrt(np.mean(e ** 2)), np.abs(e).max()))
    rms = np.mean([s[0] for s in stats]) if stats else np.nan
    worst = np.max([s[1] for s in stats]) if stats else np.nan
    return rms, worst


base = run("baseline")
prop = run("proposed")

for name, lg in (("baseline (bearing-chasing)", base),
                 ("proposed (course + crab)", prop)):
    rms, worst = metrics(lg)
    print(f"{name:28s} cross-track rms={rms:5.1f} m  worst={worst:5.1f} m  "
          f"sp flips>170deg={lg['flips']:3d}  mission done={lg['done']} "
          f"t={lg['t'][-1]:5.1f} s")

# ----------------------------------------------------------------------------
fig, axs = plt.subplots(1, 3, figsize=(17, 5.2))

ax = axs[0]
ax.plot(WAYPOINTS[:, 0], WAYPOINTS[:, 1], "k--", lw=1, label="mission legs")
ax.plot(WAYPOINTS[:, 0], WAYPOINTS[:, 1], "ks", ms=6)
ax.plot(base["pos"][:, 0], base["pos"][:, 1], "r-", lw=1.2,
        label="baseline: bearing-chasing")
ax.plot(prop["pos"][:, 0], prop["pos"][:, 1], "b-", lw=1.2,
        label="proposed: course + crab")
ax.annotate("wind + gusts", xy=(8, 38), xytext=(8, 18),
            arrowprops=dict(arrowstyle="->", color="teal", lw=2),
            color="teal", fontsize=11)
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
ax.set_title("Trajectory (same vehicle, same wind)")
ax.legend(loc="lower right", fontsize=9); ax.axis("equal")

ax = axs[1]
ax.plot(base["t"], base["e"], "r-", lw=0.9, label="baseline")
ax.plot(prop["t"], prop["e"], "b-", lw=0.9, label="proposed")
ax.axhline(0, color="gray", ls=":")
ax.set_xlabel("t (s)"); ax.set_ylabel("cross-track error (m)")
ax.set_title("Cross-track error"); ax.legend(fontsize=9)

ax = axs[2]
ax.plot(base["t"], np.degrees(base["sp"]), "r-", lw=0.7,
        label="baseline yaw sp (flips)")
ax.plot(prop["t"], np.degrees(prop["sp"]), "b-", lw=0.9,
        label="proposed yaw sp")
ax.set_xlabel("t (s)"); ax.set_ylabel("yaw setpoint (deg)")
ax.set_title("Guidance output seen by the yaw PID"); ax.legend(fontsize=9)

plt.tight_layout()
plt.savefig("guidance_demo.png", dpi=100)
print("saved guidance_demo.png")
