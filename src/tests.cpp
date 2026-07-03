// Property tests for ControlAllocator (not part of the requested deliverable,
// but they are how the allocation was verified).
//
//  1. Reconstruction: for any feasible demand, once the servos settle the
//     forward model reproduces the demand (|error| < 1e-6) — symmetric AND
//     asymmetric geometry.
//  2. Saturation: infeasible demands are scaled, not distorted — produced
//     wrench stays colinear with the demand and no thrust exceeds Tmax.
//  3. Continuity: per-tick servo increments never exceed the slew limits
//     (modulo the documented 2*pi re-base of a continuous-rotation alpha),
//     for random demand walks and adversarial sign flips.
//  4. Wrap: a force direction spinning several full turns keeps alpha
//     pose-continuous, and the stored angle stays bounded (re-based turns).
//  5. Zero-thrust freeze.
//  6. Independent inversion check against hand-computed (T, alpha, beta)
//     values (breaks the allocate()/forward() self-consistency loop).
//  7. Non-finite demands are rejected without corrupting state.
//  8. Noise-level demands do not move the servos (deadband + hysteresis).
//  9. Servo hard stops (alphaMin/alphaMax) are honored.
// 10. Thrust easing bounds the parasitic transient wrench during a reversal.
//
// Build:  g++ -std=c++17 -O2 tests.cpp -o tests

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "ControlAllocator.hpp"

using gnc::ActuatorCommand;
using gnc::AllocatorConfig;
using gnc::ControlAllocator;
using gnc::Wrench;

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}

// Deterministic LCG so runs are reproducible.
unsigned long long seed = 12345;
double urand() {  // uniform in [-1, 1]
  seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
  return static_cast<double>((seed >> 11) % 2000000) / 1000000.0 - 1.0;
}

ActuatorCommand converge(ControlAllocator& a, const Wrench& w, int n = 600) {
  ActuatorCommand c;
  for (int i = 0; i < n; ++i) c = a.allocate(w);
  return c;
}

double maxAbs(double a, double b, double c, double d) {
  return std::max(std::max(std::fabs(a), std::fabs(b)),
                  std::max(std::fabs(c), std::fabs(d)));
}

// Per-tick angle step allowing the documented 2*pi re-base jump.
double stepMod2Pi(double now, double prev) {
  const double d = now - prev;
  const double dp = std::fabs(d + 2.0 * gnc::kPi);
  const double dm = std::fabs(d - 2.0 * gnc::kPi);
  return std::min(std::fabs(d), std::min(dp, dm));
}

}  // namespace

int main() {
  AllocatorConfig cfg;
  AllocatorConfig asym;
  asym.Lf = 4.0;
  asym.Lr = 8.0;

  // --- 1. Reconstruction of feasible demands (symmetric + asymmetric) ------
  for (const AllocatorConfig& g : {cfg, asym}) {
    for (int i = 0; i < 500; ++i) {
      Wrench w{0.4 * urand(), 0.4 * urand(), 0.3 * urand(), 0.3 * urand()};
      ControlAllocator a(g);
      ActuatorCommand c = converge(a, w);
      check(c.front.T <= 1.0 + 1e-12 && c.rear.T <= 1.0 + 1e-12,
            "thrust within Tmax");
      Wrench got = ControlAllocator::forward(c, g);
      const double err =
          maxAbs(got.Fx - w.Fx, got.Fz - w.Fz, got.My - w.My, got.Mz - w.Mz);
      check(err < 1e-6, "feasible demand reconstructed exactly");
    }
  }

  // --- 2. Saturation preserves the wrench direction ------------------------
  for (int i = 0; i < 500; ++i) {
    Wrench w{2.0 * urand(), 2.0 * urand(), 1.5 * urand(), 1.5 * urand()};
    if (maxAbs(w.Fx, w.Fz, w.My, w.Mz) < 0.2) continue;
    ControlAllocator a(cfg);
    ActuatorCommand c = converge(a, w);
    check(c.front.T <= 1.0 + 1e-12 && c.rear.T <= 1.0 + 1e-12,
          "saturated thrust within Tmax");
    Wrench g = ControlAllocator::forward(c, cfg);
    // g must equal s*w for a single scalar s in (0, 1].
    const double den = w.Fx * w.Fx + w.Fz * w.Fz + w.My * w.My + w.Mz * w.Mz;
    const double s = (g.Fx * w.Fx + g.Fz * w.Fz + g.My * w.My + g.Mz * w.Mz) / den;
    check(s > 0.0 && s <= 1.0 + 1e-9, "saturation scale in (0,1]");
    check(std::fabs(a.lastScale() - s) < 1e-6, "lastScale reports the scale");
    const double err = maxAbs(g.Fx - s * w.Fx, g.Fz - s * w.Fz, g.My - s * w.My,
                              g.Mz - s * w.Mz);
    check(err < 1e-6, "saturated wrench colinear with demand");
  }

  // --- 3. Continuity under random walks and sign flips ----------------------
  {
    const double dAng = cfg.servoRateMax * cfg.dt + 1e-12;
    const double dThr = cfg.thrustRateMax * cfg.dt + 1e-12;
    ControlAllocator a(cfg);
    ActuatorCommand prev = a.allocate(Wrench{});
    Wrench w{};
    for (int i = 0; i < 20000; ++i) {
      if (i % 250 == 0) {  // adversarial step: jump to an unrelated demand
        w = Wrench{urand(), urand(), 0.8 * urand(), 0.8 * urand()};
      } else {  // small random walk
        w.Fx += 0.02 * urand();
        w.Fz += 0.02 * urand();
        w.My += 0.02 * urand();
        w.Mz += 0.02 * urand();
      }
      ActuatorCommand c = a.allocate(w);
      check(stepMod2Pi(c.front.alpha, prev.front.alpha) <= dAng &&
                stepMod2Pi(c.rear.alpha, prev.rear.alpha) <= dAng,
            "alpha slew within limit");
      check(std::fabs(c.front.beta - prev.front.beta) <= dAng &&
                std::fabs(c.rear.beta - prev.rear.beta) <= dAng,
            "beta slew within limit");
      check(std::fabs(c.front.T - prev.front.T) <= dThr &&
                std::fabs(c.rear.T - prev.rear.T) <= dThr,
            "thrust slew within limit");
      prev = c;
    }
  }

  // --- 4. alpha pose-continuous across the +/-180 deg boundary, bounded ----
  {
    ControlAllocator a(cfg);
    double prevAlpha = 0.0;
    bool first = true;
    // Force direction rotating 3 full turns in the x-z plane.
    for (int i = 0; i < 3 * 3600; ++i) {
      const double th = i * (gnc::kPi / 1800.0);
      Wrench w;
      w.Fx = 0.5 * std::sin(th);
      w.Fz = -0.5 * std::cos(th);
      ActuatorCommand c = a.allocate(w);
      if (!first)
        check(stepMod2Pi(c.front.alpha, prevAlpha) < 0.1,
              "alpha pose-continuous across wrap");
      check(std::fabs(c.front.alpha) <= 2.0 * gnc::kPi + 1e-9,
            "alpha stays bounded (turns re-based)");
      prevAlpha = c.front.alpha;
      first = false;
    }
  }

  // --- 5. Zero-thrust freeze -------------------------------------------------
  {
    ControlAllocator a(cfg);
    ActuatorCommand on = converge(a, Wrench{0.5, -0.2, 0, 0});
    ActuatorCommand off = converge(a, Wrench{});
    check(off.front.T == 0.0 && off.rear.T == 0.0, "thrust reaches zero");
    check(off.front.alpha == on.front.alpha && off.front.beta == on.front.beta,
          "angles frozen at zero thrust");
  }

  // --- 6. Independent hand-computed inversion (asymmetric geometry) ---------
  {
    // w = {0.4, -0.5, 0.2, 0.1}, Lf=4, Lr=8, computed by hand:
    //   front f = (0.4, 0.1, -13/15): T=0.959745337, a=-24.775141°, b=+5.980749°
    //   rear  f = (0.4, -0.1, -2/15): T=0.433333333, a=-71.565051°, b=-13.342364°
    ControlAllocator a(asym);
    ActuatorCommand c = converge(a, Wrench{0.4, -0.5, 0.2, 0.1});
    const double r = 180.0 / gnc::kPi;
    check(std::fabs(c.front.T - 0.959745337) < 1e-8 &&
              std::fabs(c.front.alpha * r - -24.775141) < 1e-5 &&
              std::fabs(c.front.beta * r - 5.980749) < 1e-5,
          "front pod matches hand-computed (T, alpha, beta)");
    check(std::fabs(c.rear.T - 0.433333333) < 1e-8 &&
              std::fabs(c.rear.alpha * r - -71.565051) < 1e-5 &&
              std::fabs(c.rear.beta * r - -13.342364) < 1e-5,
          "rear pod matches hand-computed (T, alpha, beta)");
  }

  // --- 7. Non-finite demands rejected without corrupting state --------------
  {
    ControlAllocator a(cfg);
    ActuatorCommand good = converge(a, Wrench{0.5, -0.3, 0.1, 0.1});
    const double nan = std::nan("");
    const double inf = 1.0 / 0.0;
    ActuatorCommand held = a.allocate(Wrench{nan, -0.3, 0.1, 0.1});
    check(held.front.T == good.front.T && held.front.alpha == good.front.alpha,
          "NaN demand holds last command");
    a.allocate(Wrench{0.5, inf, 0.1, 0.1});
    a.allocate(Wrench{0.5, -0.3, -inf, nan});
    ActuatorCommand after = converge(a, Wrench{0.5, -0.3, 0.1, 0.1});
    check(std::isfinite(after.front.T) && std::isfinite(after.front.alpha) &&
              std::isfinite(after.front.beta) && std::isfinite(after.rear.T) &&
              std::isfinite(after.rear.alpha) && std::isfinite(after.rear.beta),
          "state recovers after non-finite demands");
    Wrench g = ControlAllocator::forward(after, cfg);
    check(maxAbs(g.Fx - 0.5, g.Fz - -0.3, g.My - 0.1, g.Mz - 0.1) < 1e-6,
          "allocation exact again after non-finite episode");
  }

  // --- 8. Noise-level demands do not move the servos ------------------------
  {
    ControlAllocator a(cfg);
    converge(a, Wrench{0.5, 0, 0, 0});  // park at alpha=-90
    const double a0 = a.state().front.alpha;
    double travel = 0.0, prevA = a0;
    for (int i = 0; i < 4000; ++i) {
      // Alternating numerical dust, far below the deadband.
      Wrench w{(i % 2 ? 1.0 : -1.0) * 1e-8, 1e-8 * urand(), 0, 0};
      ActuatorCommand c = a.allocate(w);
      travel += std::fabs(c.front.alpha - prevA);
      prevA = c.front.alpha;
    }
    check(travel < 1e-12, "servos do not chase noise below the deadband");
    // Hysteresis: 1.5x deadband is not enough to unfreeze...
    converge(a, Wrench{1.5 * cfg.thrustDeadband, 0, 0, 0}, 100);
    check(a.state().front.alpha == prevA, "frozen until 2x deadband");
    // ...but a meaningful demand is.
    converge(a, Wrench{0.3, 0, 0, 0});
    check(std::fabs(a.state().front.alpha - -gnc::kPi / 2.0) < 1e-9,
          "tracking resumes for meaningful demands");
  }

  // --- 9. Servo hard stops honored -------------------------------------------
  {
    AllocatorConfig stops = cfg;
    stops.alphaMin = -120.0 * gnc::kPi / 180.0;
    stops.alphaMax = 120.0 * gnc::kPi / 180.0;
    ControlAllocator a(stops);
    // Pure nose-up moment wants the rear pod at alpha=180 deg: unreachable,
    // must clamp at the stop, never wind past it.
    for (int i = 0; i < 800; ++i) {
      ActuatorCommand c = a.allocate(Wrench{0, 0, 0.4, 0});
      check(c.rear.alpha >= stops.alphaMin - 1e-12 &&
                c.rear.alpha <= stops.alphaMax + 1e-12,
            "alpha within hard stops");
    }
  }

  // --- 10. Thrust easing bounds the parasitic transient wrench ---------------
  {
    // Cruise reversal Fx +0.6 -> -0.6: the alpha sweep passes through
    // vertical, so an un-eased pod injects a large uncommanded Fz.
    double worstEased = 0.0, worstRaw = 0.0;
    for (int mode = 0; mode < 2; ++mode) {
      AllocatorConfig e = cfg;
      e.easeThrustDuringSwing = (mode == 0);
      ControlAllocator a(e);
      converge(a, Wrench{0.6, 0, 0, 0});
      double worst = 0.0;
      for (int i = 0; i < 200; ++i) {
        Wrench g = ControlAllocator::forward(a.allocate(Wrench{-0.6, 0, 0, 0}), e);
        worst = std::max(worst, std::fabs(g.Fz));  // uncommanded vertical force
      }
      (mode == 0 ? worstEased : worstRaw) = worst;
    }
    check(worstEased < 0.6 * worstRaw,
          "easing cuts the parasitic transient force substantially");
    check(worstEased < 0.35, "eased parasitic force bounded");
  }

  if (failures == 0) std::printf("ALL TESTS PASSED\n");
  return failures == 0 ? 0 : 1;
}
