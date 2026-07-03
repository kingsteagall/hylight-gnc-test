# GNC Technical Test — Answers

David Steagall — July 2026

- Exercise 1: allocation strategy (Task 1) and C++ implementation (Task 2, `src/`).
- Exercise 2: drift analysis and guidance proposal, supported by the Annex B log (`analysis/`).

---

## Exercise 1 — Control allocation

### Task 1 — Allocation strategy

#### Conventions and actuator model

Body frame is Nose-Right-Down (Annex A): **x** forward, **y** right, **z** down.
The front gyro sits at `(+Lf, 0, 0)`, the rear gyro at `(−Lr, 0, 0)`, both on the
hull centerline. In the neutral position thrust points **up** (−z).

The exterior servo (α) is the outer gimbal and tilts the thrust in the x–z plane
(α > 0 → backward); the interior servo (β) is the inner gimbal, carried by the
outer one (β > 0 → right). Composing the two rotations (outer applied last):

```
t(α, β) = R_y(α) · R_x(β) · (0, 0, −1)
        = ( −sin α · cos β ,  sin β ,  −cos α · cos β )
```

This matches all three reference configurations of Annex A:

| α | β | t(α, β) | figure |
|---|---|---------|--------|
| 0° | 0° | (0, 0, −1) — up | neutral |
| 90° | 0° | (−1, 0, 0) — backward | "force to the back" |
| 0° | −90° | (0, −1, 0) — left | "force to the left" |

*Assumption:* the exterior servo is the outer gimbal (it carries the interior
one), which is what the gyro photo shows. The three single-angle reference
figures cannot distinguish the gimbal order — it only matters when both angles
are non-zero — so this is stated explicitly. If the order were reversed, the
inversion below swaps the roles of the two angle formulas; nothing else changes.

#### Force and moment balance

Each pod produces `f = T · t(α, β)` with `T ≥ 0` (propellers do not reverse).
With the pod position vectors `r_f = (+Lf, 0, 0)`, `r_r = (−Lr, 0, 0)` and
`M = r × f`, the demanded wrench gives five equations:

```
Fx = fx_f + fx_r                        (axial)
Fz = fz_f + fz_r                        (vertical)
0  = fy_f + fy_r                        (no net side force, required)
My = −Lf·fz_f + Lr·fz_r                 (pitch: vertical differential)
Mz = +Lf·fy_f − Lr·fy_r                 (yaw: lateral differential)
```

Both pods are on the centerline, so `Mx ≡ 0` identically — consistent with roll
being uncontrolled. Six unknowns (a 3D force per pod) minus five equations
leave **one free degree of freedom: the axial split**. I split `Fx` equally
between the pods: symmetric, predictable, and exactly minimum-peak-thrust
whenever the two pods carry equal transverse loads (`My = 0`, e.g. cruise).
When a pitch moment loads one pod more than the other, the optimal split
shifts axial force toward the less-loaded pod — still closed-form
(`u = (q_f² − q_r²) / 4Fx` equalizes the two pod norms) — but it only matters
near saturation, so I keep it as a documented refinement rather than
complicate the baseline.

The closed-form solution, with `L = Lf + Lr`:

```
fx_f = fx_r = Fx' / 2
fz_f = ( Lr·Fz' − My' ) / L          fz_r = ( Lf·Fz' + My' ) / L
fy_f =  Mz' / L                      fy_r = − Mz' / L
```

(primes denote the demands after the unit scaling below). Sanity checks: an
upward front thrust (`fz_f < 0`, z is down) alone gives `My > 0`, nose-up;
a rightward front force gives `Mz > 0`, nose-right — both physically correct.

#### Per-pod inversion

Given the desired pod force `f`, thrust and angles follow in closed form:

```
T = ‖f‖
β = asin( fy / T )                    β ∈ [−90°, +90°]
α = atan2( −fx , −fz )
```

There is no iteration and no ambiguity: over `α ∈ (−180°, 180°]`,
`β ∈ [−90°, 90°]` the map covers every thrust direction exactly once — except
at the poles `β = ±90°` (pure lateral thrust), where α is degenerate; that
singularity is handled explicitly in the continuity layer below.

#### Constraint 1 — normalized, unit-free demands

The allocator defines its own consistent scaling and expresses thrust as a
fraction of `Tmax` (so `T ∈ [0, 1]`):

- `|Fx| = 1` or `|Fz| = 1` ⇔ both motors at full thrust along that axis
  (scale `2·Tmax`).
- `|My| = 1` or `|Mz| = 1` ⇔ the full antisymmetric pair (front and rear at
  `Tmax` in opposite directions; scale `(Lf+Lr)·Tmax`).

A unit command on any single axis is therefore exactly the maximum the
airframe can do on that axis. Note the feasible set of *combined* demands is
not the unit box — the true envelope is the coupled constraint `‖f‖ ≤ Tmax`
per pod, so e.g. `Fz = −1` consumes the entire budget and leaves nothing for
`My`. That coupling is unavoidable with two thrusters; the allocator makes it
explicit rather than hiding it:

**Saturation policy.** If either pod needs `‖f‖ > Tmax`, both pod forces are
scaled down by the same factor. This preserves the *direction* of the
commanded wrench — force and moments shrink together, and no spurious moment
is created by clipping one component. The achieved fraction is exposed to the
caller (`lastScale()`) so upstream integrators can anti-windup against what
was actually allocated instead of what was requested. (A prioritized
alternative — e.g. guarantee `Fz` for altitude safety, then yaw, then axial —
drops straight into the same spot in the code as a staged allocation;
direction-preserving scaling is the predictable default and is what the
property tests pin down.)

#### Constraint 2 — no abrupt servo motion

The inversion above is memoryless; all continuity handling is a stateful
post-processing layer per pod, in this order:

1. **Direction-trust deadband with hysteresis.** Below ~0.1% of `Tmax` the
   force is physically negligible, but `atan2`/`asin` would chase its
   (noise-defined) direction at full slew rate — pure servo wear. The angles
   freeze below the deadband and only resume tracking above twice the
   deadband, so a demand dithering around the threshold cannot chatter. (A
   separate `eps ≈ 1e-9` remains as the pure divide-by-zero guard.)
2. **Singularity hold.** At `β = ±90°` (pure lateral thrust) α is undefined
   (gimbal-lock-like); α holds its previous value there.
3. **Shortest-path unwrap of α within the servo range.** The raw `atan2`
   jumps at ±180°. The commanded α is chosen among the `2π`-equivalent
   representations as the one closest to the previous command that the servo
   can reach. The default assumes a continuous-rotation joint (the stored
   angle is re-based by whole turns to stay within `(−2π, 2π]` — the same
   physical pose, so precision cannot degrade on long missions); hardware
   with mechanical stops sets `alphaMin/alphaMax` and the allocator plans
   within them, clamping unreachable targets to the nearest bound (the
   residual wrench error is then visible through the forward model — see the
   hard-stops case in the demo).
4. **Slew-rate limiting** of α, β (default 120°/s) and of T — the commanded
   trajectory never asks more of a servo than it can physically do, whatever
   the controller upstream outputs.
5. **Thrust easing during large swings** (optional, on by default). While a
   pod still points far from its commanded direction, thrusting at full power
   pushes in unwanted directions; thrust is faded with the alignment cosine
   and restored as the servo arrives. A thrust reversal thus becomes: thrust
   fades → servo sweeps 180° at its rate limit → thrust returns (demo shows
   the full sequence, ~1.5 s). Easing is per-pod, so an asymmetric maneuver
   (one pod swinging far, the other not) still produces a transient
   uncommanded wrench — bounded (each pod's force never opposes its command
   by more than 90°, and the episode lasts sweep-angle/slew-rate ≈ 1–1.5 s)
   and slow compared to airship rigid-body modes (10–30 s). The property
   tests measure it: easing cuts the parasitic transient force to less than
   60% of the un-eased case. A coordinated variant (scale both pods by the
   worse alignment) is a config-level extension if tighter transients are
   needed.

Items 4–5 encode a practical lesson: any discontinuity fed to a vectoring
actuator under load turns into a transient disturbance for the vehicle;
sweeping slowly with reduced thrust is the stable way to cross a
configuration change.

One more robustness rule, cheap and non-negotiable in flight software: a
non-finite demand (`NaN`/`Inf` from an upstream fault) is rejected and the
last command held — it must never latch into the servo state.

### Task 2 — C++ implementation

Files (plain C++17, no dependencies, header-only class):

- `src/ControlAllocator.hpp` — the `gnc::ControlAllocator` class.
  `allocate(Wrench) → ActuatorCommand` maps `(Fx, Fz, My, Mz)` to
  `(T_f, T_r, α_f, α_r, β_f, β_r)`; the instance carries the continuity state.
  A static `forward()` model reconstructs the produced wrench from a command —
  used to verify the allocation. Accessors: `state()` (current pose, for
  telemetry) and `lastScale()` (achieved demand fraction, for anti-windup).
  The constructor sanitizes the config (geometry/timing must be positive) and
  `allocate()` rejects non-finite demands by holding the last command.
- `src/demo.cpp` — the demonstration script requested by the task: steady-state
  demands (climb, cruise, reverse, pure pitch, pure yaw, combined, saturating
  with the achieved-percentage report), config variants (asymmetric arms,
  servo hard stops), NaN robustness, and the two continuity showcases (cruise
  reversal with thrust easing, zero-thrust freeze). Every case prints the
  command *and* the wrench reconstructed by the forward model.
- `src/tests.cpp` — property tests: exact reconstruction of 1 000 random
  feasible demands (symmetric and asymmetric geometry); colinearity +
  `T ≤ Tmax` + `lastScale()` correctness under 500 random saturating demands;
  slew-limit compliance over 20 000 ticks of random walks with adversarial
  demand jumps; α pose-continuity and boundedness across three full turns; an
  independent hand-computed `(T, α, β)` check that breaks the
  `allocate()/forward()` self-consistency loop; NaN/Inf rejection and
  recovery; noise-level demands moving the servos by exactly zero; hard-stop
  compliance; and a measurement that thrust easing cuts the parasitic
  transient force during a reversal.

Build and run:

```
g++ -std=c++17 -O2 src/demo.cpp  -o demo   && ./demo
g++ -std=c++17 -O2 src/tests.cpp -o tests  && ./tests
```

All tests pass (`ALL TESTS PASSED`). Representative demo output:

```
cruise+climb+yaw             | cmd F=(+0.40,-0.40) M=(+0.00,+0.30)
  front: T=0.640 a=  -45.0 b= +27.9   rear: T=0.640 a=  -45.0 b= -27.9
  produced F=(+0.40,-0.40) M=(+0.00,+0.30)

saturating demand            | cmd F=(+1.00,-1.00) M=(+0.50,+0.50)
  produced F=(+0.53,-0.53) M=(+0.27,+0.27)     <- scaled, direction preserved
```

Design notes an interviewer may ask about:

- **Why closed-form instead of an optimizer (pseudo-inverse / QP)?** The
  system is small and structured; the exact solution is 10 lines, runs in
  nanoseconds, has no convergence corner cases, and its failure mode
  (saturation) is handled explicitly. On an embedded target determinism wins.
- **Why is the class stateful?** Continuity is a property *between* successive
  commands; something has to remember the previous servo pose. Keeping it in
  the allocator keeps the flight controller stateless with respect to
  actuator geometry.
- **Real-time suitability:** no heap allocation, no STL containers, `noexcept`
  API, O(1) per call (a dozen scalar libm calls). Double precision throughout;
  porting to `float` is mechanical (all math is scalar) but requires retuning
  tolerances — the turn re-basing already keeps angles small enough for float
  resolution.

---

## Exercise 2 — Mission mode

### Q1 — Why the airship drifts off the straight line

*Evidence below from the Annex B log (365 s: altitude → mission 33–263 s →
position 263–351 s → altitude; extraction script and plots in `analysis/`).*

**Physical causes**

1. **No lateral actuation** (`Fy = 0` by design): any lateral disturbance is
   uncontrollable directly; the only cure is pointing the hull.
2. **Wind and gusts**: the flight shows a sustained velocity deficit in cruise
   (setpoint 3.0 m/s, achieved 2.2 m/s) and, in position mode, gusts push the
   ship **backwards to −2.4 m/s** while station-keeping. A crosswind component
   translates directly into sideslip: course over ground ≠ heading.
3. **Slender-body directional instability (Munk moment)**: the bare hull is
   statically unstable in yaw; in cruise the yaw error hunts with a ~30 s
   period, RMS ≈10° and excursions to ~41° (130–260 s window, setpoint
   wrap-flips masked) — classic heading hunting under gusts with a PID that
   has no disturbance model. Every degree of heading error at cruise speed is
   ≈4 cm/s of lateral velocity, so the hunting alone injects 0.4 m/s RMS and
   up to ~1.4 m/s of cross-track drift rate.
4. **Large inertia, low yaw authority**: the waypoint turnaround at t≈85–130 s
   takes ~45 s for a net 261° rotation (~5.8°/s average). Long yaw transients
   mean long stretches flown with the nose off-course.

**Algorithmic causes**

5. **The controller tracks *heading*, not *course*.** The yaw PID follows its
   setpoint reasonably well when the setpoint is sane — and the ship still
   drifts, because in wind the velocity vector is not where the nose points.
   Nothing in the loop observes or closes the actual error that matters.
6. **Bearing-to-waypoint guidance (pure pursuit) converges to the point, not
   to the line.** Recomputing bearing from a drifting position yields the
   classic pursuit curve that bows downwind; cross-track error is never
   penalized.
7. **Yaw setpoint wrap bug**: the logged setpoint flips between +180° and
   −180° **22 times** (t≈85–125 s and t≈245 s) when the desired heading sits
   near south. Fed to a PID without shortest-arc error handling, each flip
   commands a full-circle turn; the log indeed shows the vehicle winding
   through ±180° repeatedly during that window.
8. **And the loop cannot even observe the error it needs to close.** Annex B
   contains vx, altitude, pitch, yaw and their setpoints, but **no lateral
   position, no vy, no course over ground, no cross-track error, no actuator
   commands, no wind estimate**. The drift cannot even be *seen* in this log,
   only inferred — which is the answer to whether the logs "contain enough
   information": not quite, and that gap is itself part of the problem.
   Altitude and pitch meanwhile track with RMS ≈0.75 m / ≈0.9° (peaks 1.6 m /
   4.2°) — the problem is confined to the horizontal plane, and the quantity
   that defines it is not measured, not logged, and not controlled.

### Q2 — Proposal: control the track, not the nose

Realistic with the Exercise 1 actuators (yaw moment + axial thrust only,
bounded authority, no perfect wind knowledge):

1. **Add a guidance layer that measures cross-track error.** Project the GNSS
   position onto the current leg (segment WPᵢ→WPᵢ₊₁): along-track distance `s`
   and cross-track error `e`. This is pure software; the sensors already exist.
2. **Command course, not heading.** Convert the leg into a desired *course*:
   `course_cmd = leg_azimuth + correction(e)`, with the correction bounded
   (±30–45°) — an L1/lookahead law or simply a PD on `e`. Boundedness is what
   makes it honest: in wind stronger than the achievable crab, the residual
   drift is bounded and known, not assumed away.
3. **Let integral action find the crab angle.** The yaw setpoint becomes
   `yaw_sp = course_cmd + crab`, where the crab term is the slow integral of
   the cross-track error (or of the course-minus-heading difference). This
   implicitly estimates the wind triangle without a wind sensor and without
   assuming perfect cancellation — steady wind ⇒ steady crab; gusts ⇒ bounded
   transients.
4. **Fix the yaw pipeline while at it**: unwrap the setpoint and compute PID
   error as the shortest arc (`wrap(ψ_sp − ψ)`); this removes the 22 wrap
   flips and the commanded full circles for free. Switch waypoints by
   crossing the perpendicular plane at the waypoint (or an acceptance radius)
   with hysteresis, instead of chasing a bearing that swings wildly at close
   range — that eliminates the 45 s turnaround thrash seen in the log.
5. **Adapt speed to wind.** The crab angle needed grows with
   `asin(wind_lat / airspeed)`: when the achievable crab saturates, slow down
   the *along-track* setpoint rather than give up the line (the same
   trade-off the log shows the vehicle making, involuntarily, when gusts
   push it backwards). Faster flight reduces the required crab but excites
   the Munk moment; the guidance should pick the speed, not fight it.
6. **Log what the loop needs**: cross-track error, vy/course over ground, and
   actuator commands — both to close the loop and to make the next flight
   debuggable.

Expected result — from having implemented this scheme in my own 6-DoF airship
flight simulator (twin thrust-vectoring pods fore/aft, no aerodynamic
surfaces, comparable to this vehicle): switching from bearing-chasing to
course-based guidance with an integral crab term took cross-track drift in
gusty crosswind from ±20 m to ≈10 m, with yaw error bounded and no
waypoint-switch spins. The residual is bounded drift, not zero drift — which
is what the actuator set can honestly deliver.

---

## Appendix — log analysis method

The Annex B HTML embeds Plotly traces as base64 binary buffers.
`analysis/extract_log.py` decodes them (9 traces, 50–100 ms sampling),
computes per-mode tracking statistics and produces `analysis/flight_overview.png`
and `analysis/flight_zoom.png` referenced above.
