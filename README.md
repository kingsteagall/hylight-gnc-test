# HyLight GNC technical test — solution

David Roberts Steagall — 03/07/2026

- [ANSWERS.md](ANSWERS.md) — the answers document (Exercise 1 strategy, Exercise 2 analysis);
  rendered as `deliverable/GNC-Technical-Test-Answers.pdf`.
- `src/ControlAllocator.hpp` — Exercise 1 Task 2: the allocation class (header-only, C++17, no deps).
- `src/demo.cpp` — required demonstration script.
- `src/tests.cpp` — property tests used to verify the allocation.
- `analysis/` — Annex B flight-log extraction, guidance comparison and plots.
- `deliverable/` — the PDF and the scripts that build it.

## Building Exercise 1

```
g++ -std=c++17 -O2 src/demo.cpp  -o demo   &&  ./demo
g++ -std=c++17 -O2 src/tests.cpp -o tests  &&  ./tests    # → ALL TESTS PASSED
```

(`build.sh` does the same via em++/Node on a machine without native g++.)

## Running the analysis (Exercise 2)

- `python analysis/guidance_demo.py` — **self-contained** (numpy + matplotlib
  only); reproduces the planar baseline-vs-proposed comparison and
  `guidance_demo.png` in ~1 s. This is the reproducible demonstration of the
  Q2 result (prints: baseline rms 29.8 m / worst 57.4 m; proposed rms 7.5 m /
  worst 28.4 m).
- `python analysis/extract_log.py <path to mission_hylight.html>` — reproduces
  every Annex B statistic quoted in the answers plus `flight_overview.png` /
  `flight_zoom.png`. The Annex B HTML is the reviewer's own file and is **not
  shipped** in this package.
- `python analysis/plot_sim_validation.py analysis/ex2-results.json` —
  regenerates `sim_validation.png` from the included result files.

**Honest boundary:** the full-simulator numbers (the 2.6–3.2× cross-track
reduction, 4-seed table) come from a separate, private simulator repository;
they ship here as **data** (`analysis/ex2-results-{42,7,123,2026}.json` +
`sim_validation.png`), and `analysis/sim-ex2-guidance.mjs.txt` is reference
source only (it runs inside that repo). `analysis/guidance_demo.py` is the
standalone, package-reproducible version of the same result.

Test statement (`GNC - Technical test.pdf`) and flight log
(`mission_hylight.html`) are the reviewer's files and are not committed.
