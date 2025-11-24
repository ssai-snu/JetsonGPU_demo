# Jetson CUDA Programming Examples (Jetson NX / TX2)

This repository contains CUDA sample projects adapted for **NVIDIA
Jetson platforms** (Jetson Nano, TX2, Xavier NX, Orin).\
It includes two full GPU simulation demos:

-   **N-body Simulation** (CUDA compute + OpenGL rendering)\
-   **Smoke Particles Simulation** (CUDA + GLSL + OpenGL)

Both versions are provided for: - **Jetson Xavier NX** →
`CudaProgramming_nx/` - **Jetson TX2** → `CudaProgramming_tx2/`

A shared CUDA utility library is included in `common/`.

------------------------------------------------------------------------

## 📁 Project Structure

    CudaProgramming_nx/
        nbody/              # CUDA N-body simulation
        smokeParticles/      # CUDA-based smoke particle renderer

    CudaProgramming_tx2/
        nbody/
        smokeParticles/

    common/
        inc/                 # CUDA helper headers
        src/                 # Rendering + threading utils
        data/                # Sample images

Each project includes: - `Makefile` - CUDA sources (`.cu`, `.cuh`) -
OpenGL renderer (`.cpp`) - Data files (`.bin`, `.ppm`) - Documentation
PDFs and screenshots

------------------------------------------------------------------------

## 🚀 Build Instructions (Jetson Linux)

### 1. Install CUDA & OpenGL dev packages

JetPack already includes CUDA, but install OpenGL/EGL:

``` bash
sudo apt-get update
sudo apt-get install freeglut3-dev libglew-dev libglm-dev
```

### 2. Build N-body

``` bash
cd CudaProgramming_nx/nbody
make -j4
```

### 3. Build SmokeParticles

``` bash
cd CudaProgramming_nx/smokeParticles
make -j4
```

(Replace `nx` with `tx2` when building for Jetson TX2)

------------------------------------------------------------------------

## ▶️ Running the demos

### N-body simulation

``` bash
./nbody
```

### Smoke Particles simulation

``` bash
./particleDemo
```

Both demos open an OpenGL window and use CUDA for GPU simulation.

------------------------------------------------------------------------

## 🧩 Included Features

### ✔ N-body Simulation

-   Direct CUDA gravitational interaction computation\
-   Shared memory optimization\
-   OpenGL VBO rendering\
-   Sample dataset: `galaxy_20K.bin`

### ✔ Smoke Particles

-   CUDA-based particle advection\
-   3D texture sampling\
-   GLSL-based rendering\
-   Framebuffer object (FBO) pipeline\
-   Reference velocity fields included

### ✔ Common CUDA Utilities

-   `helper_cuda.h`, `helper_math.h`
-   GLEW/GLUT support
-   nvMath, nvMatrix, nvQuaternion, nvVector
-   Basic multithreading + timers

------------------------------------------------------------------------

## 🛠 Requirements

-   NVIDIA Jetson (TX2, Xavier NX, Nano, Orin)
-   JetPack (CUDA Toolkit + OpenGL)
-   GCC/G++\
-   Make

------------------------------------------------------------------------

## 📌 Notes

-   Precompiled object files (`*.o`) are included but can be removed if
    rebuilding from scratch.
-   Both projects are adapted from NVIDIA CUDA SDK samples.
