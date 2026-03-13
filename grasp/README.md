# C++ implementation of Game4Grasp. 

The C++ code is organized into several source files and includes a Python wrapper for easy integration with Python-based applications.

## 0. Prerequisites

To build this project, you will need the following dependencies installed on your system:

- CMake ≥ 2.8.3 (for configuring and generating the build system)

- C++17-compatible compiler (e.g., g++ ≥ 7 or clang++ ≥ 5)

- Eigen3 (linear algebra library)

- Ceres Solver (nonlinear optimization library)

- pybind11 (for Python bindings, if you want to use the wrapper)

Make sure they are installed and their include/library paths are discoverable by CMake.

## 1. Configure & build

```bash
# from the repo root
mkdir -p grasp/build
cd grasp/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```
### What gets built

- A static library: libGrasp.a (from source/*.cpp)

- One executable per test file in test/*.cpp (auto-discovered by CMake)

## 2. Run tests

```bash
# still inside grasp/build
./test_grasp
```

## 3. CMake hints (if dependencies aren’t found)

Point CMake to Ceres/Eigen installs if they’re not on default paths:

```bash
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="/path/to/ceres;/another/path" \
  -DEigen3_DIR="/path/to/eigen/cmake"
```

## 4. Clean rebuild

```bash
rm -rf grasp/build && mkdir -p grasp/build && cd grasp/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```