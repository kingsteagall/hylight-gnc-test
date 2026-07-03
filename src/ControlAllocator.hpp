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
//   Thrusts below are expressed as fractions of Tmax, so T in [0,1].
//
// Continuity (constraint 2): stateful post-processing per pod —
//   freeze angles at zero thrust, shortest-path unwrap of alpha, slew-rate
//   limiting of alpha/beta/T, and optional thrust easing while the servos
//   are still far from the commanded direction (a swinging pod at full
//   thrust sprays transient forces in unwanted directions).

#include <algorithm>
#include <cmath>

namespace gnc {

constexpr double kPi = 3.14159265358979323846;

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
  double Lf = 6.0;              // CG -> front gyro arm [m]
  double Lr = 6.0;              // CG -> rear gyro arm [m]
  double dt = 0.02;             // command period [s]
  double servoRateMax = 2.09;   // max servo slew [rad/s] (~120 deg/s)
  double thrustRateMax = 2.0;   // max thrust slew [Tmax/s] (0 -> full in 0.5 s)
  bool easeThrustDuringSwing = true;
  double eps = 1e-9;            // zero-thrust threshold
};

inline double clamp(double v, double lo, double hi) {
  return std::min(std::max(v, lo), hi);
}

// Wrap angle to (-pi, pi].
inline double wrapPi(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a <= 0.0) a += 2.0 * kPi;
  return a - kPi;
}

class ControlAllocator {
 public:
  explicit ControlAllocator(const AllocatorConfig& cfg = AllocatorConfig())
      : cfg_(cfg) {}

  // Map one normalized wrench demand to actuator commands.
  // Stateful: successive calls are continuous in the servo angles.
  ActuatorCommand allocate(const Wrench& w) {
    const double L = cfg_.Lf + cfg_.Lr;

    // 1) Wrench -> desired 3D force per pod, in thrust units (Tmax = 1).
    //    Fx = fxf + fxr           (axial: the one free DOF, split equally)
    //    Fz = fzf + fzr           My = -Lf*fzf + Lr*fzr   (vertical pair)
    //    0  = fyf + fyr           Mz =  Lf*fyf - Lr*fyr   (lateral pair)
    double fxf = w.Fx, fxr = w.Fx;
    double fzf = 2.0 * cfg_.Lr / L * w.Fz - w.My;
    double fzr = 2.0 * cfg_.Lf / L * w.Fz + w.My;
    double fyf = w.Mz, fyr = -w.Mz;

    // 2) Saturation: uniform down-scaling preserves the direction of the
    //    commanded wrench (F and M shrink together, no cross-coupling).
    const double nf = std::sqrt(fxf * fxf + fyf * fyf + fzf * fzf);
    const double nr = std::sqrt(fxr * fxr + fyr * fyr + fzr * fzr);
    const double over = std::max(1.0, std::max(nf, nr));
    fxf /= over; fyf /= over; fzf /= over;
    fxr /= over; fyr /= over; fzr /= over;

    // 3) Per-pod inversion + continuity layer.
    front_ = podStep(front_, fxf, fyf, fzf);
    rear_ = podStep(rear_, fxr, fyr, fzr);
    return ActuatorCommand{front_, rear_};
  }

  // Forward model: normalized wrench actually produced by a command.
  // Used by the demo and the property tests to verify the allocation.
  static Wrench forward(const ActuatorCommand& c, const AllocatorConfig& cfg) {
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

  const AllocatorConfig& config() const { return cfg_; }

  // Reset the continuity state (e.g. on arming) to a known servo pose.
  void reset(const PodCommand& front = PodCommand(),
             const PodCommand& rear = PodCommand()) {
    front_ = front;
    rear_ = rear;
  }

 private:
  // Desired force vector -> (T, alpha, beta) with continuity handling.
  PodCommand podStep(const PodCommand& prev, double fx, double fy, double fz) const {
    const double T = std::sqrt(fx * fx + fy * fy + fz * fz);

    PodCommand tgt;
    if (T < cfg_.eps) {
      // Zero thrust: direction undefined, hold the servos where they are.
      tgt.alpha = prev.alpha;
      tgt.beta = prev.beta;
      tgt.T = 0.0;
    } else {
      // beta from the y component, alpha from the (-x,-z) projection.
      tgt.beta = std::asin(clamp(fy / T, -1.0, 1.0));
      const double px = -fx, pz = -fz;
      if (px * px + pz * pz < cfg_.eps * cfg_.eps) {
        // Pure lateral thrust: alpha undefined (gimbal singularity), hold it.
        tgt.alpha = prev.alpha;
      } else {
        // Shortest-path unwrap: stay on the turn count closest to prev.alpha.
        const double raw = std::atan2(px, pz);
        tgt.alpha = prev.alpha + wrapPi(raw - prev.alpha);
      }
      tgt.T = T;
    }

    // Slew-rate limiting: what a real servo/motor can do in one period.
    PodCommand out;
    const double dAng = cfg_.servoRateMax * cfg_.dt;
    const double dThr = cfg_.thrustRateMax * cfg_.dt;
    out.alpha = prev.alpha + clamp(tgt.alpha - prev.alpha, -dAng, dAng);
    out.beta = prev.beta + clamp(tgt.beta - prev.beta, -dAng, dAng);
    double Tcmd = tgt.T;

    if (cfg_.easeThrustDuringSwing && tgt.T >= cfg_.eps) {
      // While the pod still points far from the commanded direction, thrusting
      // at full power pushes sideways; fade thrust with the alignment cosine.
      const double cosErr = dir(out.alpha, out.beta).dot(dir(tgt.alpha, tgt.beta));
      Tcmd = tgt.T * clamp(cosErr, 0.0, 1.0);
    }
    out.T = clamp(prev.T + clamp(Tcmd - prev.T, -dThr, dThr), 0.0, 1.0);
    return out;
  }

  struct Vec3 {
    double x, y, z;
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  };
  static Vec3 dir(double alpha, double beta) {
    return Vec3{-std::sin(alpha) * std::cos(beta), std::sin(beta),
                -std::cos(alpha) * std::cos(beta)};
  }

  AllocatorConfig cfg_;
  PodCommand front_;
  PodCommand rear_;
};

}  // namespace gnc
