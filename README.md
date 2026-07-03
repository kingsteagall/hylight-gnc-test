# HyLight GNC technical test — submission workspace

- [ANSWERS.md](ANSWERS.md) — the answers document (Exercise 1 strategy, Exercise 2 analysis).
- `src/ControlAllocator.hpp` — Exercise 1 Task 2: the allocation class (header-only, C++17, no deps).
- `src/demo.cpp` — required demonstration script.
- `src/tests.cpp` — property tests used to verify the allocation.
- `analysis/` — Annex B flight-log extraction script and plots.
- `build.sh` — local build/run (uses em++/Node because this machine has no
  native g++; the sources build unmodified with `g++ -std=c++17 -O2`).

Test statement: `GNC - Technical test.pdf` (not committed). Flight log:
`mission_hylight.html` (not committed, 5 MB).
