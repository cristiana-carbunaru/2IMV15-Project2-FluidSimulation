# 2IMV15 Project 2 - Coupled Fluid-Rigid Simulation

**Group 7**: Komal Jha, Cristiana Cărbunaru, Konstantin Georgiev  
Course: 2IMV15 Simulation in Computer Graphics  
Project: Project 2 - Fluid Simulation

This project extends the already-submitted Project 1 particle/cloth framework with a new interactive **Project 2 fluid scene**. The program starts in the fluid scene by default, while the original Project 1 scenes remain available by pressing `s`.

Project 2's implementation is based on Jos Stam's Stable Fluids solver structure, and adds the required fluid-solid features from the assignment: vorticity confinement, fixed grid-edge obstacles, moving solid objects, rotating rigid bodies, collision impulses, two-way fluid-solid coupling, tracer particles, and a small cloth patch driven by the velocity field. The final version also includes the optional temperature/buoyancy and free-surface water features.

## Implemented Features

| Feature | Main files | Runtime control |
|---|---|---|
| Stable Fluids base solver: semi-Lagrangian advection, implicit diffusion, pressure projection, Gauss-Seidel relaxation | `FluidSolver2D.*` | Always active |
| Vorticity confinement | `FluidSolver2D::applyVorticityConfinement` | `z` |
| Fixed internal objects / arbitrary grid-edge boundaries | `FluidSolver2D.*`, `FluidScene::rebuildSolids` | `f` |
| Moving solid objects dragged by the user | `FluidScene::mouse`, `FluidScene::motion`, `RigidBody2D.*` | `m` |
| Rotating rigid bodies | `RigidBody2D::surfaceVelocityAt`, `FluidScene.*` | `r` |
| Collision impulses between rigid bodies | `FluidScene::handleRigidCollisions`, `RigidBody2D::applyImpulse` | `b` |
| Two-way fluid-solid coupling | `FluidScene::applyFluidForcesToBodies` | `o` |
| Particles and cloth interacting with the fluid | `FluidScene::stepTracers`, `FluidScene::stepCloth` | `p` |
| Temperature and thermal buoyancy | `FluidSolver2D::applyBuoyancy` | `t` |
| Free-surface / water mode with marker particles and zero-pressure air interface | `FluidSolver2D::initWater`, `FluidSolver2D::project`, `FluidScene::drawWaterParticles` | `w` |
| Multi-field visualization: density, velocity, vorticity, pressure, temperature | `FluidScene::draw` | `v` |
| PNG frame dumping | `imageio.*`, `ParticleToy.cpp` | `d` |

## Project Structure

```text
2IMV15-Project2/
├── bin/                        # Executable output folder and Windows DLLs
├── external/CDROM_GDC03_original/
    └── ...                     # Preserved original Stam GDC reference archive
├── include/common/             # Shared headers
│   ├── FluidSolver2D.h         # Stable Fluids grid solver and scalar/vector fields
│   ├── FluidScene.h            # Project 2 scene controller and UI state
│   ├── RigidBody2D.h           # Simple box/circle rigid body representation
│   └── ...                     # Project 1 particle/cloth headers
├── include/windows/            # Windows library headers from the original setup
├── include/linux/              # Linux library headers from the original setup
├── lib/                        # 32-bit course libraries
├── lib/x64/                    # 64-bit freeglut libraries for CLion/MinGW
├── obj/                        # Intermediate object files generated during compilation
├── snapshots/                  # PNG frame dumps are written here
├── src/
│   ├── FluidSolver2D.cpp       # Fluid solver, boundaries, vorticity, buoyancy, water
│   ├── FluidScene.cpp          # Demo scene, controls, coupling, particles, cloth, drawing
│   ├── RigidBody2D.cpp         # Rigid-body geometry, velocities, impulses, drawing
│   ├── ParticleToy.cpp         # Application entry point and Project 1/2 scene switching
│   └── ...                     # Project 1 source files
├── CMakeLists.txt              # CLion/CMake build configuration
├── compile_linux.txt   # Linux compilation instructions
├── compile_mac.txt     # macOS compilation instructions
├── compile_win10.txt   # Windows compilation instructions
├── Makefile                    # Terminal make build configuration
└── README.md                   # This file
```

The `external/CDROM_GDC03_original/` folder is kept only for traceability. It contains Stam's original reference material (can be downloaded from: http://www.dgp.toronto.edu/people/stam/reality/Research/zip/CDROM_GDC03.zip), including old PocketPC/Palm binaries. Those binaries are not compiled or used by this project.

## Building and Running

### Option A: Build in CLion / CMake on Windows

1. Open this project folder in CLion.
2. Open the project by selecting the top-level `CMakeLists.txt` file.
3. Use a MinGW toolchain. The project was tested with CLion's bundled 64-bit MinGW.
4. Build the `project2` target.
5. In `Run -> Edit Configurations...`, set the working directory to the project root, for example:

```text
D:\...\2IMV15-Project2
```
**Important**: Do not use `bin/` or `cmake-build-debug/` as the working directory, otherwise screenshot dumping will not find the `snapshots/` folder.

6. Run `project2` from CLion.

The executable is written to:

```text
bin/project2.exe
```

### Option B: Build with Makefile

From the project root, open a Terminal window and run:

```bash
make clean
make
```

Then run:

```bash
./bin/project2
```

On Windows PowerShell, run:

```powershell
.\bin\project2.exe
```

The Makefile uses an explicit source list. This avoids linker errors caused by accidentally compiling multiple files that contain their own `main()` function.

### Linux/macOS Dependencies

Install OpenGL and GLUT/freeGLUT development packages. On Linux, this is typically:

```bash
sudo apt install freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev
```

On macOS, Xcode command line tools are usually sufficient for the GLUT/OpenGL framework path used by the Makefile/CMake file.

## Project 2 Runtime Controls

The simulation starts paused. Press `space` to run/pause.

| Input | Action |
|---|---|
| `space` | Run/pause simulation |
| Left-drag empty fluid | Inject velocity into the fluid |
| Left-drag a rigid body | Move the body through the fluid; on release it snaps to the grid |
| Right-drag | Add smoke/density and heat |
| `v` | Cycle view: density, velocity, vorticity, pressure, temperature |
| `z` | Toggle vorticity confinement |
| `f` | Toggle fixed internal boundaries |
| `m` | Toggle moving solid objects |
| `r` | Toggle rigid-body rotation |
| `b` | Toggle rigid-body collision impulses |
| `o` | Toggle two-way fluid/solid coupling |
| `p` | Toggle tracer particles and cloth |
| `t` | Toggle temperature buoyancy |
| `g` | Toggle gravity on rigid bodies |
| `w` | Toggle free-surface water / marker-particle mode |
| `c` | Clear and reset the fluid fields |
| `d` | Toggle PNG frame dumping to `snapshots/` |
| `h` | Print Project 2 controls in the console |
| `s` | Cycle between the original Project 1 scenes and the Project 2 scene |
| `q` | Quit |

Most Project 2 features are enabled by default in the final demo scene so that the default view already shows the complete coupled system. The toggles are included so that the demo video can isolate individual effects.

## Screenshots and frame dumping

Press `d` while the desired simulation window is focused. The program writes PNG files to:

```text
snapshots/img00000.png
snapshots/img00001.png
...
```

If the console prints a message such as `Could not dump snapshots/img00000.png`, check that:

1. the `snapshots/` folder exists, and/or
2. the CLion working directory is the project root.

## Notes on PNG support

The original Project 1 code used `imageio.cpp` with `libpng12`. Project 2 keeps the same `imageio.h` / `imageio.cpp` interface and still writes PNG files. For 64-bit CLion builds, `imageio.cpp` uses a built-in PNG-writing path to avoid the old 32-bit `libpng12.a` linker mismatch. Compatible 32-bit/course toolchains can still use the original libpng path by defining `IMAGEIO_USE_LIBPNG`.

## Notes on the solver and limitations

The solver keeps Stam's simple cell-centred grid layout for readability and compatibility with the original GDC 2003 code. This is enough for the course demo, but a staggered MAC grid would give more accurate pressure/boundary handling around moving solids. The rigid-body collision model uses impulse-based separation and a restitution threshold, and it does not implement a full resting-contact solver. The free-surface water feature uses marker particles and a boolean water-cell mask, so it is visually useful, but does not strictly conserve volume in the way a full particle level-set or FLIP solver would.

## References used for the implementation

- Course slides on Stable Fluids, boundary conditions, rigid-body dynamics, and rigid-body collisions.
- ENRIGHT, D., MARSCHNER, S., AND FEDKIW, R. 2002. Animation and Rendering of Complex Water Surfaces. In Computer Graphics (SIGGRAPH 2002), ACM, 736-744.
- FEDKIW, R., STAM, J., AND JENSEN, H. 2001. Visual Simulation of Smoke. In Computer Graphics (SIGGRAPH 2001), ACM, 15-22.
- STAM, J. 1999. Stable Fluids. In Computer Graphics (SIGGRAPH 1999), ACM, 121-128.
- STAM, J. 2003. Real-Time Fluid Dynamics for Games. Proceedings of the Game Developer Conference.
- The original Project 1 particle/cloth framework.

## License
This project is intended for educational purposes.
