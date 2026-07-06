# RANSAC Plane Fitting — From Scratch (Python & C++) + Benchmark

The ground-segmentation step of the main pipeline calls Open3D's `segment_plane`.
This sub-project reimplements that RANSAC plane fit **from scratch** — first in
Python (no `segment_plane`), then ported to C++ with hand-written vector math (no
Eigen / no linear-algebra solver) — and benchmarks all three on the **same cloud**.

The goal is not to beat Open3D, but to show the algorithm is understood from
first principles *and* can be moved into a low-level, hot-loop implementation —
the "prototype in Python, push the hot loop to C++" workflow.

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
that inner loop is the "hot loop" being measured.

## Benchmark

Same cloud, same parameters for all three methods.

- **Data:** Autzen (`autzen.laz`), voxel-downsampled at `voxel_size = 5.0`
  → **1,389,751 points**.
- **Parameters:** `distance_threshold = 15.0`, `ransac_n = 3`,
  `num_iterations = 1000`.
- Only the RANSAC loop is timed (file I/O excluded). C++ compiled with `-O2`.
- Machine: Apple Silicon (macOS), single-threaded for the from-scratch versions.

| Method                         | Time (ms) | Speedup vs pure Python |
|--------------------------------|-----------|------------------------|
| Pure Python (NumPy-vectorized) | 5493      | 1.0×                   |
| **C++ (from scratch)**         | **1258**  | **4.4×**               |
| Open3D (`segment_plane`)       | 189       | 29×                    |

All three converge to the same plane (normal ≈ `(0, 0, 1)`, ~765k–792k inliers),
so the comparison is apples-to-apples on correctness.

### Reading the numbers honestly

- The Python baseline is **already NumPy-vectorized**, i.e. its hot loop runs in C.
  So the fair comparison here is *vectorized NumPy vs hand-written C++* — hence
  **~4.4×**, not the 50–100× you'd see against a naive triple-nested-loop Python.
- Open3D is ~6.7× faster than the hand-written C++. That gap is expected: Open3D's
  implementation is multi-threaded and SIMD-optimized, while this C++ port is
  single-threaded scalar code. Getting a from-scratch, single-threaded port to
  within ~7× of a production library is the point.

## Build & run

```bash
# 1. Export the downsampled cloud from the Python pipeline (writes cpp/data/cloud.csv)
python cpp/export_cloud.py

# 2. Pure-Python + Open3D baseline (prints both, plus timings)
python cpp/ransac_scratch.py

# 3. C++ version (-O2 is required for a fair benchmark)
clang++ -O2 -std=c++17 cpp/ransac.cpp -o cpp/ransac
./cpp/ransac
```

Run from the repository root so the relative paths resolve.
`cpp/data/cloud.csv` (~99 MB) is generated and git-ignored.

## Files

- `ransac_scratch.py` — from-scratch RANSAC in Python (vectorized) + Open3D baseline.
- `export_cloud.py` — writes the downsampled cloud to `cpp/data/cloud.csv`.
- `ransac.cpp` — from-scratch RANSAC in C++ (`struct Point`, `std::vector`,
  hand-written cross/dot/norm, `std::mt19937`, `std::chrono` with `steady_clock`).

## Next steps (not yet done)

- **Optimize the C++ port** to close the gap with Open3D: drop the per-point
  division (compare `|n·p + d| < threshold · |n|`), `-O3 -march=native`, then
  multi-thread the iterations (OpenMP / `std::thread`).
- **pybind11**: expose the C++ RANSAC as a Python module and drop it into the main
  pipeline in place of `segment_plane`.
