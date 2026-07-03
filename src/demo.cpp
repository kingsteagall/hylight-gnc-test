// Exercise 1, Task 2 — demonstration of the ControlAllocator class.
//
// Runs a set of representative demands, lets the allocator converge (the
// continuity layer slew-limits the servos), then prints the actuator
// commands and the wrench actually produced by the forward model.
//
// Build:  g++ -std=c++17 -O2 demo.cpp -o demo   (or em++/clang++, header-only)

#include <cstdio>

#include "ControlAllocator.hpp"

using gnc::ActuatorCommand;
using gnc::AllocatorConfig;
using gnc::ControlAllocator;
using gnc::Wrench;

namespace {

double deg(double rad) { return rad * 180.0 / gnc::kPi; }

void printCase(const char* name, const Wrench& w, const ActuatorCommand& c,
               const AllocatorConfig& cfg) {
  const Wrench got = ControlAllocator::forward(c, cfg);
  std::printf("%-28s | cmd F=(%+5.2f,%+5.2f) M=(%+5.2f,%+5.2f)\n", name, w.Fx,
              w.Fz, w.My, w.Mz);
  std::printf(
      "  front: T=%.3f a=%+7.1f deg b=%+6.1f deg   rear: T=%.3f a=%+7.1f deg "
      "b=%+6.1f deg\n",
      c.front.T, deg(c.front.alpha), deg(c.front.beta), c.rear.T,
      deg(c.rear.alpha), deg(c.rear.beta));
  std::printf("  produced F=(%+5.2f,%+5.2f) M=(%+5.2f,%+5.2f)\n\n", got.Fx,
              got.Fz, got.My, got.Mz);
}

// Run the same demand until the slew-limited servos settle.
ActuatorCommand converge(ControlAllocator& alloc, const Wrench& w,
                         int ticks = 400) {
  ActuatorCommand c;
  for (int i = 0; i < ticks; ++i) c = alloc.allocate(w);
  return c;
}

}  // namespace

int main() {
  AllocatorConfig cfg;
  ControlAllocator alloc(cfg);

  std::printf("== Steady-state demands (allocator converged) ==\n\n");

  struct Case {
    const char* name;
    Wrench w;
  } cases[] = {
      {"neutral (all zero)", {0, 0, 0, 0}},
      {"climb (Fz=-0.6, z is down)", {0, -0.6, 0, 0}},
      {"cruise (Fx=+0.5)", {0.5, 0, 0, 0}},
      {"reverse (Fx=-0.3)", {-0.3, 0, 0, 0}},
      {"pitch nose-up (My=+0.4)", {0, 0, 0.4, 0}},
      {"yaw right (Mz=+0.4)", {0, 0, 0, 0.4}},
      {"cruise+climb+yaw", {0.4, -0.4, 0, 0.3}},
      {"saturating demand", {1.0, -1.0, 0.5, 0.5}},
  };
  for (const Case& k : cases) {
    alloc.reset();
    printCase(k.name, k.w, converge(alloc, k.w), cfg);
    if (alloc.lastScale() < 1.0)
      std::printf("  (demand infeasible: allocated %.0f%%, direction preserved)\n\n",
                  100.0 * alloc.lastScale());
  }

  std::printf("== Config: asymmetric arms (Lf=4, Lr=8) ==\n\n");
  AllocatorConfig acf;
  acf.Lf = 4.0;
  acf.Lr = 8.0;
  ControlAllocator asym(acf);
  printCase("cruise+climb+pitch+yaw", {0.4, -0.5, 0.2, 0.1},
            converge(asym, {0.4, -0.5, 0.2, 0.1}), acf);

  std::printf("== Config: servo hard stops (alpha within +-120 deg) ==\n\n");
  AllocatorConfig scf;
  scf.alphaMin = -120.0 * gnc::kPi / 180.0;
  scf.alphaMax = 120.0 * gnc::kPi / 180.0;
  ControlAllocator stops(scf);
  printCase("pitch nose-up (My=+0.4)", {0, 0, 0.4, 0},
            converge(stops, {0, 0, 0.4, 0}), scf);

  std::printf("== Robustness: non-finite demand is held, then recovers ==\n\n");
  alloc.reset();
  converge(alloc, {0.5, 0, 0, 0});
  ActuatorCommand held = alloc.allocate({0.0 / 0.0, 0, 0, 0});
  std::printf("  NaN demand -> last command held (T=%.3f a=%+.1f deg), state clean\n\n",
              held.front.T, deg(held.front.alpha));

  std::printf("== Continuity: cruise reversal Fx +0.8 -> -0.8 ==\n\n");
  alloc.reset();
  converge(alloc, {0.8, 0, 0, 0});
  std::printf("  t[s]   T_f    alpha_f[deg]  (thrust eases while the servo "
              "sweeps ~180 deg)\n");
  const Wrench rev{-0.8, 0, 0, 0};
  for (int i = 0; i <= 100; ++i) {
    ActuatorCommand c = alloc.allocate(rev);
    if (i % 10 == 0)
      std::printf("  %4.1f  %.3f  %+8.1f\n", i * cfg.dt, c.front.T,
                  deg(c.front.alpha));
  }

  std::printf("\n== Continuity: zero-thrust freeze ==\n\n");
  alloc.reset();
  ActuatorCommand before = converge(alloc, {0.5, 0, 0, 0});
  ActuatorCommand after = converge(alloc, {0, 0, 0, 0});
  std::printf("  thrust off: T=%.3f, alpha held at %+.1f deg (was %+.1f)\n",
              after.front.T, deg(after.front.alpha), deg(before.front.alpha));

  return 0;
}
