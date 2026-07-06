# RANSAC Plane Fitting — From Scratch (Python & C++) + Benchmark

The ground-segmentation step of the main pipeline calls Open3D's `segment_plane`.
This sub-project reimplements that RANSAC plane fit **from scratch** — first in
Python (no `segment_plane`), then ported to C++ with hand-written vector math (no
Eigen / no linear-algebra solver), then optimized (OpenMP) — and benchmarks
everything on the **same cloud**.

The goal is not to beat Open3D, but to show the algorithm is understood from
first principles *and* can be moved into a low-level, hot-loop implementation and
then profiled/optimized — the "prototype in Python, push the hot loop to C++,
then make it fast" workflow.

## The algorithm

RANSAC = randomly sample, then vote. Repeat `num_iterations` times:

1. Pick 3 distinct random points `p0, p1, p2`.
2. Two edge vectors: `v1 = p1 - p0`, `v2 = p2 - p0`.
3. Plane normal via **cross product**: `n = v1 × v2` → `(a, b, c)`.
4. Plane equation `ax + by + cz + d = 0`, with `d = -(n · p0)`.
5. Distance of any point `p`: `|n · p + d| / |n|`.
6. Count points with distance `< threshold` (inliers); keep the best model.

The Python version vectorizes the distance computation with NumPy (all points in
one operation); the C++ version walks every point in an explicit inner loop —
that inner loop is the "hot loop" being measured and optimized.

## Benchmark

Same cloud, same parameters for all methods.

- **Data:** Autzen (`autzen.laz`), voxel-downsampled at `voxel_size = 5.0`
  → **1,389,751 points**.
- **Parameters:** `distance_threshold = 15.0`, `ransac_n = 3`,
  `num_iterations = 1000`.
- Only the RANSAC loop is timed (file I/O excluded). C++ compiled with `-O3`.
- **Machine:** Apple Silicon (macOS), **4 performance + 4 efficiency cores**.

| Method                              | Time (ms) | Speedup vs pure Python |
|-------------------------------------|-----------|------------------------|
| Pure Python (NumPy-vectorized)      | 5493      | 1.0×                   |
| C++ single-thread (scalar, `-O2`)   | 1258      | 4.4×                   |
| **C++ optimized (OpenMP, 4 threads)** | **475**   | **11.6×**              |
| Open3D (`segment_plane`)            | 189       | 29×                    |

All methods converge to the same plane (normal ≈ `(0, 0, 1)`, ~765k–790k
inliers), so the comparison is apples-to-apples on correctness.

### Optimizations applied to the C++ port

- **Removed the per-point division.** Instead of `|n·p + d| / |n| < threshold`,
  compare `|n·p + d| < threshold · |n|` — `|n|` is computed once per hypothesis,
  killing 1.4M divisions per iteration.
- **`-O3 -march=native`** for aggressive auto-vectorization.
- **OpenMP.** The 1000 hypotheses are split across threads; each thread has its
  own `mt19937` (seeded per thread) and its own local best model, merged in a
  `#pragma omp critical` section at the end. Total work stays 1000 hypotheses, so
  the comparison remains fair.
- Tried `schedule(dynamic)` — no material gain (the kernel is memory-bandwidth-
  bound, so scheduling is a secondary lever).

### Parallel scaling (median of 5 runs, optimized binary)

| Threads | Median time (ms) | Speedup vs 1 thread |
|---------|------------------|---------------------|
| 1       | 996              | 1.0×                |
| 2       | 726              | 1.37×               |
| 4       | **475**          | **2.10×**           |
| 8       | 536              | 1.86× (regresses)   |

**Reading the curve honestly:**

- Even on 4 cores the speedup is only **2.1×, not 4×** — the inner loop streams
  the entire ~33 MB point array from memory every iteration (~33 GB of traffic
  over 1000 iterations). It is **memory-bandwidth-bound**, not compute-bound;
  cores share the memory bus, so more cores ≠ proportional speedup.
- **8 threads is slower than 4.** The 4 efficiency cores add contention on an
  already-saturated memory bus without adding usable bandwidth, and become
  stragglers. The sweet spot is 4 threads (the performance cores).
- Single-thread time alone varies **~1.0–1.3 s** run to run, depending on whether
  the OS parks the thread on a performance or an efficiency core — the ~1.3×
  P/E-core speed ratio is directly visible as measurement noise.
- The parallel version is **non-deterministic across thread counts** (per-thread
  RNG seeds), so the inlier count shifts slightly between runs (765k–790k); the
  recovered plane is the same.

The takeaway is not "8× faster" but *why* it is not: a from-scratch,
single-threaded scalar port reaches ~4.4× over vectorized NumPy; optimization
(no-divide + `-O3` + OpenMP) reaches ~11.6×, and the remaining gap to Open3D's
189 ms is a memory-bandwidth wall on a heterogeneous-core machine.

## Build & run

```bash
# 1. Export the downsampled cloud from the Python pipeline (writes cpp/data/cloud.csv)
python cpp/export_cloud.py

# 2. Pure-Python + Open3D baseline (prints both, plus timings)
python cpp/ransac_scratch.py

# 3. C++ version — needs libomp on macOS (Apple Clang has no built-in OpenMP)
brew install libomp
clang++ -O3 -march=native -std=c++17 \
  -Xpreprocessor -fopenmp \
  -I$(brew --prefix libomp)/include -L$(brew --prefix libomp)/lib -lomp \
  cpp/ransac.cpp -o cpp/ransac

OMP_NUM_THREADS=4 ./cpp/ransac    # sweet spot on a 4P+4E machine

# scaling sweep
for t in 1 2 4 8; do echo -n "threads=$t: "; OMP_NUM_THREADS=$t ./cpp/ransac | grep Measured; done
```

Run from the repository root so the relative paths resolve.
`cpp/data/cloud.csv` (~99 MB) is generated and git-ignored.

## Files

- `ransac_scratch.py` — from-scratch RANSAC in Python (vectorized) + Open3D baseline.
- `export_cloud.py` — writes the downsampled cloud to `cpp/data/cloud.csv`.
- `ransac.cpp` — from-scratch RANSAC in C++ (`struct Point`, `std::vector`,
  hand-written cross/dot/norm, `std::mt19937`, `std::chrono` with `steady_clock`,
  OpenMP-parallelized with per-thread RNG and a `critical` merge).

## Next steps (not yet done)

- **pybind11**: expose the C++ RANSAC as a Python module and drop it into the main
  pipeline in place of `segment_plane`.
