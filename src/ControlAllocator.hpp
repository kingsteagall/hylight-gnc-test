#pragma once
// Control allocation for a twin-gyro airship (GNC technical test, Exercise 1).
//
// Conventions (Annex A):
//   Body frame: Nose-Right-Down (x forward, y right, z down).
//   Gyro neutral position points thrust UP, i.e. along -z.
//   alpha (exterior servo, outer gimbal about body-y): positive tilts thrust BACKWARD (-x).
//   beta  (interior servo, inner gimbal):              positive tilts thrust RIGHT    (+y).
//
// Thrust direction unit vector (outer gimbal applied last):
//   t(alpha, beta) = R_y(alpha) * R_x(beta) * (0,0,-1)
//                  = ( -sin(alpha)*cos(beta),  sin(beta),  -cos(alpha)*cos(beta) )
// Checks against the three Annex A reference figures:
//   (0,0)     -> (0,0,-1)  up       (neutral)
//   (90°,0)   -> (-1,0,0)  backward
//   (0,-90°)  -> (0,-1,0)  left
//
// Normalized command scaling (constraint 1: controller outputs carry no units):
//   |Fx| = 1 or |Fz| = 1  <-> both motors at full thrust along that axis (scale 2*Tmax).
//   |My| = 1 or |Mz| = 1  <-> full antisymmetric thrust pair (scale (Lf+Lr)*Tmax).
//   Thrusts below are expressed as fractions of Tmax, so T in [0,1]. Note the
//   feasible set of combined demands is NOT the unit box: each pod's 3D force
//   norm is capped at Tmax, which couples the axes (see saturation policy).
//
// Continuity (constraint 2): stateful post-processing per pod —
//   angle freeze below a thrust deadband (with hysteresis), shortest-path
//   unwrap of alpha within the configured servo range, slew-rate limiting of
//   alpha/beta/T, and optional thrust easing while the servos are still far
//   from the commanded direction (a swinging pod at full thrust sprays
//   transient forces in unwanted directions).
//
// Servo travel assumption: by default alpha is treated as a continuous-
//   rotation joint (slip-ring); the emitted angle is re-based by whole turns
//   to stay within (-2*pi, 2*pi] — the same physical pose — so state cannot
//   grow without bound on long missions. Hardware with mechanical stops sets
//   alphaMin/alphaMax and the allocator plans within them.

#include <algorithm>
#include <cmath>
#include <limits>

namespace gnc {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

// Normalized controller demand. Fy = 0 and Mx uncontrolled by problem statement.
struct Wrench {
  double Fx = 0.0;
  double Fz = 0.0;
  double My = 0.0;
  double Mz = 0.0;
};

struct PodCommand {
  double T = 0.0;      // thrust, fraction of Tmax, in [0,1]
  double alpha = 0.0;  // exterior servo angle [rad]
  double beta = 0.0;   // interior servo angle [rad]
};

struct ActuatorCommand {
  PodCommand front;
  PodCommand rear;
};

struct AllocatorConfig {
  double Lf = 6.0;               // CG -> front gyro arm [m]
  double Lr = 6.0;               // CG -> rear gyro arm [m]
  double dt = 0.02;              // fixed command period [s]
  double servoRateMax = 2.09;    // max servo slew [rad/s] (~120 deg/s)
  double thrustRateMax = 2.0;    // max thrust slew [Tmax/s] (0 -> full in 0.5 s)
  double alphaMin = -kInf;       // exterior servo stops [rad]; +-inf = continuous
  double alphaMax = kInf;
  bool easeThrustDuringSwing = true;
  double thrustDeadband = 1e-3;  // direction-trust threshold [Tmax]; below it
                                 // the angles freeze (hysteresis: unfreeze at 2x)
  double eps = 1e-9;             // pure divide-by-zero guard
};

inline double clamp(double v, double lo, double hi) noexcept {
  // std::clamp is UB for lo > hi; this returns hi instead (safe under
  // misconfiguration, which the constructor also sanitizes against).
  return std::min(std::max(v, lo), hi);
}

// Wrap angle to (-pi, pi].
inline double wrapPi(double a) noexcept {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a <= 0.0) a += 2.0 * kPi;
  return a - kPi;
}

class ControlAllocator {
 public:
  explicit ControlAllocator(const AllocatorConfig& cfg = AllocatorConfig()) noexcept
      : cfg_(cfg) {
    // Sanitize: a silently-invalid config must not become NaN/garbage at
    // flight rate. Fall back to defaults for anything non-physical.
    const AllocatorConfig d;
    if (!(cfg_.Lf > 0.0) || !std::isfinite(cfg_.Lf)) cfg_.Lf = d.Lf;
    if (!(cfg_.Lr > 0.0) || !std::isfinite(cfg_.Lr)) cfg_.Lr = d.Lr;
    if (!(cfg_.dt > 0.0) || !std::isfinite(cfg_.dt)) cfg_.dt = d.dt;
    if (!(cfg_.servoRateMax >= 0.0)) cfg_.servoRateMax = d.servoRateMax;
    if (!(cfg_.thrustRateMax >= 0.0)) cfg_.thrustRateMax = d.thrustRateMax;
    if (!(cfg_.thrustDeadband > 0.0)) cfg_.thrustDeadband = d.thrustDeadband;
    if (!(cfg_.eps > 0.0)) cfg_.eps = d.eps;
    if (!(cfg_.alphaMin < cfg_.alphaMax)) {
      cfg_.alphaMin = d.alphaMin;
      cfg_.alphaMax = d.alphaMax;
    }
  }

  // Map one normalized wrench demand to actuator commands.
  // Stateful: successive calls are continuous in the servo angles.
  // Must be called at the fixed period cfg.dt for the slew limits to hold.
  ActuatorCommand allocate(const Wrench& w) noexcept {
    if (!(std::isfinite(w.Fx) && std::isfinite(w.Fz) && std::isfinite(w.My) &&
          std::isfinite(w.Mz))) {
      // Non-finite demand: hold the last command. An upstream numerical
      // fault must not latch into the servo state (recoverable next tick).
      return ActuatorCommand{front_, rear_};
    }

    const double L = cfg_.Lf + cfg_.Lr;

    // 1) Wrench -> desired 3D force per pod, in thrust units (Tmax = 1).
    //    With the normalization above the demanded totals are Fx' = 2*Fx,
    //    Fz' = 2*Fz, My' = L*My, Mz' = L*Mz, and the balance
    //      Fx' = fxf + fxr           Fz' = fzf + fzr
    //      My' = -Lf*fzf + Lr*fzr    Mz' = Lf*fyf - Lr*fyr    0 = fyf + fyr
    //    solved with the axial force split equally (the free DOF) gives:
    double fxf = w.Fx, fxr = w.Fx;
    double fzf = 2.0 * cfg_.Lr / L * w.Fz - w.My;
    double fzr = 2.0 * cfg_.Lf / L * w.Fz + w.My;
    double fyf = w.Mz, fyr = -w.Mz;

    // 2) Saturation: uniform down-scaling preserves the direction of the
    //    commanded wrench (F and M shrink together, no cross-coupling).
    //    The achieved fraction is exposed via lastScale() so an upstream
    //    integrator can anti-windup against real actuation.
    const double nf = std::sqrt(fxf * fxf + fyf * fyf + fzf * fzf);
    const double nr = std::sqrt(fxr * fxr + fyr * fyr + fzr * fzr);
    const double over = std::max(1.0, std::max(nf, nr));
    scale_ = 1.0 / over;
    fxf *= scale_; fyf *= scale_; fzf *= scale_;
    fxr *= scale_; fyr *= scale_; fzr *= scale_;

    // 3) Per-pod inversion + continuity layer.
    front_ = podStep(front_, fxf, fyf, fzf);
    rear_ = podStep(rear_, fxr, fyr, fzr);
    return ActuatorCommand{front_, rear_};
  }

  // Forward model: normalized wrench actually produced by a command.
  // Used by the demo and the property tests to verify the allocation.
  static Wrench forward(const ActuatorCommand& c, const AllocatorConfig& cfg) noexcept {
    const double L = cfg.Lf + cfg.Lr;
    const double fxf = -c.front.T * std::sin(c.front.alpha) * std::cos(c.front.beta);
    const double fyf = c.front.T * std::sin(c.front.beta);
    const double fzf = -c.front.T * std::cos(c.front.alpha) * std::cos(c.front.beta);
    const double fxr = -c.rear.T * std::sin(c.rear.alpha) * std::cos(c.rear.beta);
    const double fyr = c.rear.T * std::sin(c.rear.beta);
    const double fzr = -c.rear.T * std::cos(c.rear.alpha) * std::cos(c.rear.beta);
    Wrench w;
    w.Fx = (fxf + fxr) / 2.0;
    w.Fz = (fzf + fzr) / 2.0;
    w.My = (-cfg.Lf * fzf + cfg.Lr * fzr) / L;
    w.Mz = (cfg.Lf * fyf - cfg.Lr * fyr) / L;
    return w;
  }

  const AllocatorConfig& config() const noexcept { return cfg_; }

  // Current servo/motor pose (for telemetry/logging without stepping).
  ActuatorCommand state() const noexcept { return ActuatorCommand{front_, rear_}; }

  // Fraction of the last demand actually allocated (1 = no saturation).
  double lastScale() const noexcept { return scale_; }

  // Reset the continuity state (e.g. on arming) to a known servo pose.
  void reset(const PodCommand& front = PodCommand(),
             const PodCommand& rear = PodCommand()) noexcept {
    front_ = front;
    rear_ = rear;
    scale_ = 1.0;
  }

 private:
  // Desired force vector -> (T, alpha, beta) with continuity handling.
  PodCommand podStep(const PodCommand& prev, double fx, double fy, double fz) const noexcept {
    const double T = std::sqrt(fx * fx + fy * fy + fz * fz);

    // Direction-trust deadband with hysteresis: below ~0.1% Tmax the force is
    // physically negligible but atan2/asin would chase its (noise-defined)
    // direction at full slew rate — pure servo wear. Track the direction only
    // for meaningful demands; when frozen, require 2x the deadband to resume.
    const bool wasTracking = prev.T >= cfg_.thrustDeadband;
    const bool track =
        T >= (wasTracking ? cfg_.thrustDeadband : 2.0 * cfg_.thrustDeadband);

    PodCommand tgt;
    tgt.T = T;
    if (!track) {
      tgt.alpha = prev.alpha;
      tgt.beta = prev.beta;
    } else {
      // beta from the y component, alpha from the (-x,-z) projection
      // (cos(beta) >= 0 over beta's range licenses the factorization).
      tgt.beta = std::asin(clamp(fy / T, -1.0, 1.0));
      const double px = -fx, pz = -fz;
      if (px * px + pz * pz < cfg_.eps * cfg_.eps) {
        // Pure lateral thrust: alpha undefined (gimbal singularity), hold it.
        tgt.alpha = prev.alpha;
      } else {
        // Shortest-path unwrap within the servo range: take the 2*pi-
        // equivalent representation closest to prev that the servo can
        // reach; with stops, an out-of-range target clamps to the bound.
        const double raw = std::atan2(px, pz);
        double a = prev.alpha + wrapPi(raw - prev.alpha);
        if (a > cfg_.alphaMax) {
          const double alt = a - 2.0 * kPi;
          a = (alt >= cfg_.alphaMin) ? alt : cfg_.alphaMax;
        } else if (a < cfg_.alphaMin) {
          const double alt = a + 2.0 * kPi;
          a = (alt <= cfg_.alphaMax) ? alt : cfg_.alphaMin;
        }
        tgt.alpha = a;
      }
    }

    // Slew-rate limiting: what a real servo/motor can do in one period.
    PodCommand out;
    const double dAng = cfg_.servoRateMax * cfg_.dt;
    const double dThr = cfg_.thrustRateMax * cfg_.dt;
    out.alpha = prev.alpha + clamp(tgt.alpha - prev.alpha, -dAng, dAng);
    out.beta = prev.beta + clamp(tgt.beta - prev.beta, -dAng, dAng);
    double Tcmd = tgt.T;

    if (cfg_.easeThrustDuringSwing && track) {
      // While the pod still points far from the commanded direction, thrusting
      // at full power pushes sideways; fade thrust with the alignment cosine.
      const double cosErr = dir(out.alpha, out.beta).dot(dir(tgt.alpha, tgt.beta));
      Tcmd = tgt.T * clamp(cosErr, 0.0, 1.0);
    }
    out.T = clamp(prev.T + clamp(Tcmd - prev.T, -dThr, dThr), 0.0, 1.0);

    // Continuous-rotation default: re-base whole turns so the stored angle
    // stays within (-2*pi, 2*pi] — identical physical pose, and double/float
    // resolution cannot degrade on long missions. (Skipped with real stops.)
    if (cfg_.alphaMin == -kInf && cfg_.alphaMax == kInf) {
      if (out.alpha > 2.0 * kPi) out.alpha -= 2.0 * kPi;
      else if (out.alpha <= -2.0 * kPi) out.alpha += 2.0 * kPi;
    }
    return out;
  }

  struct Vec3 {
    double x, y, z;
    double dot(const Vec3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }
  };
  static Vec3 dir(double alpha, double beta) noexcept {
    return Vec3{-std::sin(alpha) * std::cos(beta), std::sin(beta),
                -std::cos(alpha) * std::cos(beta)};
  }

  AllocatorConfig cfg_;
  PodCommand front_;
  PodCommand rear_;
  double scale_ = 1.0;
};

}  // namespace gnc
