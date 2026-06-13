# 2IMV15 Project 2 - Fluid Simulation with Coupled Solids

This repository extends the submitted Project 1 particle/cloth code with a new **Project 2 fluid scene**. The original Project 1 scenes are still available by pressing `s`, but the program now starts in the fluid scene by default.

The Project 2 scene implements a 2D Stable Fluids solver based on Jos Stam's solver structure, plus vorticity confinement, grid-edge solid boundaries, moving and rotating rigid bodies, collision impulses, approximate two-way fluid/solid coupling, tracer particles, and a small cloth patch that is driven by the velocity field.

## Added source files

- `include/common/GLCompat.h` - cross-platform GLUT include wrapper.
- `include/common/FluidSolver2D.h`, `src/FluidSolver2D.cpp` - Stable Fluids solver and grid-edge/object boundaries.
- `include/common/RigidBody2D.h`, `src/RigidBody2D.cpp` - simple box/circle rigid bodies, rotation, impulses, and drawing.
- `include/common/FluidScene.h`, `src/FluidScene.cpp` - Project 2 demo scene, user interaction, coupling, particles, and cloth.
- `CMakeLists.txt` - CLion-friendly build file.
- `docs/Project2_Implementation_Notes.md` - explanation notes for the group.
- `external/CDROM_GDC03_original/` - unmodified reference copy of Stam's CDROM_GDC03 archive, including the old PocketPC/Palm binaries. These are kept only for traceability; the CLion/CMake target does not build or run them.

## Optional: Build in CLion

1. Open this project folder in CLion.
2. Make sure OpenGL and GLUT/freeGLUT are available.
   - macOS: Xcode command line tools are usually enough for GLUT/OpenGL.
   - Linux: install packages such as `freeglut3-dev`, `libgl1-mesa-dev`, and `libglu1-mesa-dev`.
   - Windows/CLion: this package uses the included `lib/x64/libfreeglut_static.a` for 64-bit MinGW. No libpng12 link is required for the Project 2 fluid demo.
3. Let CLion load `CMakeLists.txt`.
4. Build the `project2` target.
5. Run the executable produced in `bin/`.

## Build with Makefile

From the project root, open a Terminal window and run:

```bash
make clean
make
./bin/project2
```


## Troubleshooting: `multiple definition of main`

A C/C++ executable may contain exactly one `main(...)` function. If the linker prints messages such as:

```text
multiple definition of `main'
obj/headless_test.o ... first defined here
obj/main.o ... first defined here
```

it means the build is compiling more than one entry-point file. In this project, `src/ParticleToy.cpp` is the real application entry point. Files such as `src/main.cpp` or `src/headless_test.cpp` are not part of the simulator target. The included Makefile and CMake file therefore use an explicit source list instead of `src/*.cpp`.

If you copied files over an older folder, either delete stale files such as `src/main.cpp` and `src/headless_test.cpp`, or replace your Makefile/CMakeLists.txt with the versions from this package and run:

```bash
make clean
make
```

## Project 2 Controls

The simulation starts paused. Press `space` to run/pause.

| Input | Action |
|---|---|
| Left drag empty fluid | Inject velocity into the fluid |
| Left drag a red/orange rigid object | Move it through the fluid; on release it snaps to the grid and stops |
| Right drag | Add smoke/density |
| `v` | Toggle density view / velocity view |
| `z` | Toggle vorticity confinement |
| `f` | Toggle fixed internal boundaries |
| `m` | Toggle moving solid objects |
| `r` | Toggle rigid-body rotation |
| `b` | Toggle collision impulses between rigid bodies |
| `o` | Toggle two-way coupling from fluid to bodies |
| `p` | Toggle tracer particles and cloth |
| `c` | Clear fluid fields |
| `h` | Print fluid controls |
| `s` | Cycle through original Project 1 scenes and the Project 2 scene |
| `q` | Quit |

## Notes and limitations

This implementation is intended as a course-project implementation, not as a production CFD solver. It keeps Stam's simple cell-centred grid layout for compatibility with the original GDC code, then approximates solid boundaries with marked grid edges and solid cells. A staggered/MAC grid would give cleaner pressure boundary conditions, especially around moving objects, but would require a larger rewrite. The rigid-body collision system uses simple impulse-based separation, matching the assignment statement that resting contact is not required.


## About the original CDROM_GDC03 executables

The original `CDROM_GDC03.zip` archive contains four old `PocketPC2002/*.exe` demo programs and one Palm `.prc` binary. They are now preserved in `external/CDROM_GDC03_original/` together with Stam's original `code/demo.c` and `code/solver.c`.

Those binaries are **not** part of the Project 2 build. They are precompiled historical demos from the reference archive, while this project is compiled from the C++ source files in `src/` and `include/common/`. Keeping them in `external/` makes the starting point complete without mixing old executables into the CLion target.

## Source/reference mapping

The code is commented with the reference source for each major technique:

- Stam 1999 / Stam GDC 2003: semi-Lagrangian advection, implicit diffusion, pressure projection, Gauss-Seidel relaxation, and the solver step order.
- Fedkiw, Stam, and Jensen 2001: vorticity confinement force `f_conf = epsilon * h * (N x omega)`; in this real-time grid solver the user-facing `vorticityStrength` absorbs the constant scale.
- Course stable-fluid and boundary-condition slides: pressure projection and solid boundary handling.
- Course rigid-body and collision slides: rigid state, surface velocity, collision impulse, and simple separation.
- Project 1 code: particle and cloth/mass-spring structures reused inside the fluid scene.

### Frame dumping / screenshots

The old Project 1 code used `imageio.cpp` and libpng12 for PNG screenshots. Project 2 keeps the same `imageio.h` / `imageio.cpp` interface and still writes `.png` screenshots into the `snapshots/` folder. The difference is that 64-bit CLion builds use a built-in PNG writer inside `imageio.cpp` instead of linking the old 32-bit `libpng12.a`. Compatible 32-bit/course toolchains can still use the original libpng path by defining `IMAGEIO_USE_LIBPNG`.

## PNG screenshot support in Project 2

Project 2 keeps the Project 1 `imageio.h` / `imageio.cpp` interface and frame-dump workflow.  Pressing the existing frame-dump control writes PNG images to `snapshots/img00000.png`, `snapshots/img00001.png`, etc.

The implementation now has two PNG back ends:

- **CLion / 64-bit MinGW default:** uses a built-in RGBA PNG writer in `src/imageio.cpp`.  This avoids linking the old 32-bit `libpng12.a` library that ships with the original course bundle and caused CLion linker errors such as `skipping incompatible libpng12.a`.
- **Original Project 1-style libpng path:** available for compatible 32-bit/course toolchains by defining `IMAGEIO_USE_LIBPNG`.  With the Makefile, run `make USE_LIBPNG=1`.  With CMake, the 32-bit Windows path and non-Windows system-libpng path enable this automatically when suitable.

This keeps PNG output while making the project build reliably in CLion.
