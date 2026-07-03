// Property tests for ControlAllocator (not part of the requested deliverable,
// but they are how the allocation was verified).
//
//  1. Reconstruction: for any feasible demand, once the servos settle the
//     forward model reproduces the demand (|error| < 1e-6).
//  2. Saturation: infeasible demands are scaled, not distorted — produced
//     wrench stays colinear with the demand and no thrust exceeds Tmax.
//  3. Continuity: per-tick servo increments never exceed the slew limits,
//     for random demand walks and for adversarial sign flips.
//  4. Wrap: a force direction spinning several full turns keeps alpha
//     continuous (shortest-path unwrap, no +/-180 deg jumps).
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

}  // namespace

int main() {
  AllocatorConfig cfg;

  // --- 1. Reconstruction of feasible demands -------------------------------
  for (int i = 0; i < 500; ++i) {
    Wrench w{0.4 * urand(), 0.4 * urand(), 0.3 * urand(), 0.3 * urand()};
    ControlAllocator a(cfg);
    ActuatorCommand c = converge(a, w);
    check(c.front.T <= 1.0 + 1e-12 && c.rear.T <= 1.0 + 1e-12,
          "thrust within Tmax");
    Wrench g = ControlAllocator::forward(c, cfg);
    const double err = maxAbs(g.Fx - w.Fx, g.Fz - w.Fz, g.My - w.My, g.Mz - w.Mz);
    check(err < 1e-6, "feasible demand reconstructed exactly");
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
    double s = 0.0;
    double den = w.Fx * w.Fx + w.Fz * w.Fz + w.My * w.My + w.Mz * w.Mz;
    s = (g.Fx * w.Fx + g.Fz * w.Fz + g.My * w.My + g.Mz * w.Mz) / den;
    check(s > 0.0 && s <= 1.0 + 1e-9, "saturation scale in (0,1]");
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
      check(std::fabs(c.front.alpha - prev.front.alpha) <= dAng &&
                std::fabs(c.rear.alpha - prev.rear.alpha) <= dAng,
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

  // --- 4. alpha stays continuous across the +/-180 deg boundary ------------
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
        check(std::fabs(c.front.alpha - prevAlpha) < 0.1,
              "alpha continuous across wrap");
      prevAlpha = c.front.alpha;
      first = false;
    }
    check(std::fabs(prevAlpha) > 4.0 * gnc::kPi,
          "alpha accumulated turns instead of jumping");
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

  if (failures == 0) std::printf("ALL TESTS PASSED\n");
  return failures == 0 ? 0 : 1;
}
