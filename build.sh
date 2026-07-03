#!/usr/bin/env bash
# Local build/run. This machine has no native g++/MSVC, so we compile with
# Emscripten (em++) and run under Node — the code itself is plain C++17 and
# builds identically with g++/clang++: g++ -std=c++17 -O2 src/demo.cpp -o demo
set -e
cd "$(dirname "$0")"

if ! command -v em++ >/dev/null 2>&1; then
  source /c/emsdk/emsdk_env.sh >/dev/null 2>&1 || source C:/emsdk/emsdk_env.sh
fi

mkdir -p out
em++ -std=c++17 -O2 src/demo.cpp -o out/demo.js
em++ -std=c++17 -O2 src/tests.cpp -o out/tests.js
echo "== demo =="
node out/demo.js
echo "== tests =="
node out/tests.js
