#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <gfx/vec2.h>

// FluidSolver2D:
// A compact, cell-centred 2D Stable Fluids solver. The structure intentionally
// stays close to Jos Stam's public GDC 2003 solver (`solver.c`): arrays store
// density and the two velocity components, and one time step uses
// add_source -> diffuse -> project -> advect -> project for velocity and
// add_source -> diffuse -> advect for density.

// Project 2 additions are placed in this class instead of being hidden inside
// the OpenGL scene so the numerical method is easy to inspect:
//   * vorticity confinement from Fedkiw, Stam, and Jensen's smoke paper,
//   * fixed internal walls represented by blocked grid edges,
//   * moving solid cells with a prescribed surface velocity,
//   * advection path clipping so backtraced samples do not pass through solids,
//   * velocity/density sampling used by tracers, cloth, and two-way coupling.

// Important simplification: this is not a staggered MAC solver. To match Stam's
// starter code and the Project 1 framework, velocities are stored at cell
// centres. The solid boundary handling is therefore an educational approximation
// suitable for the assignment demo, not a production CFD implementation.

class FluidSolver2D {
public:
    FluidSolver2D();
    explicit FluidSolver2D(int n);

    void resize(int n);
    void clear();
    // Reset only the temporary source arrays. Mouse/body forces are accumulated
    // here before each step and cleared at the end of FluidSolver2D::step.
    void clearSources();

    // Remove fixed/moving internal boundaries and solid cells. The outer domain
    // walls are immediately restored, because Stam's solver assumes a closed box.
    void clearSolids();

    int gridSize() const { return m_N; }
    int ix(int i, int j) const { return i + (m_N + 2) * j; }
    int size() const { return (m_N + 2) * (m_N + 2); }

    void setDiffusion(float value) { m_diffusion = std::max(0.0f, value); }
    void setViscosity(float value) { m_viscosity = std::max(0.0f, value); }
    void setVorticityStrength(float value) { m_vorticityStrength = std::max(0.0f, value); }
    void enableVorticity(bool enabled) { m_useVorticity = enabled; }
    bool vorticityEnabled() const { return m_useVorticity; }

    // Temperature/buoyancy (optional "Temperature" feature, Fedkiw/Stam/Jensen 2001).
    // The temperature field is transported just like density; the buoyancy force
    // f = (-alpha*density + beta*(T - Tambient)) * up is added to the vertical
    // velocity so warm smoke rises and dense smoke sinks.
    void setTemperatureDiffusion(float value) { m_temperatureDiffusion = std::max(0.0f, value); }
    void setAmbientTemperature(float value) { m_ambientTemperature = value; }
    void setBuoyancy(float alpha, float beta) { m_buoyancyAlpha = std::max(0.0f, alpha); m_buoyancyBeta = std::max(0.0f, beta); }
    void enableBuoyancy(bool enabled) { m_useBuoyancy = enabled; }
    bool buoyancyEnabled() const { return m_useBuoyancy; }

    // Add source terms. The amount is written into the *_Prev arrays because
    // Stam's solver treats those arrays as temporary source buffers.
    void addDensityCell(int i, int j, float amount);
    void addVelocityCell(int i, int j, float amountU, float amountV);
    void addDensityAt(float x, float y, float amount, int radius = 1);
    void addVelocityAt(float x, float y, float amountU, float amountV, int radius = 1);

    // Inject heat. Like density, the amount is written into the temperature
    // source buffer and applied during the next step.
    void addTemperatureCell(int i, int j, float amount);
    void addTemperatureAt(float x, float y, float amount, int radius = 1);

    // Edge walls. wallX(i,j) blocks the vertical edge between cells (i-1,j) and (i,j).
    // wallY(i,j) blocks the horizontal edge between cells (i,j-1) and (i,j).
    void setVerticalWall(int i, int j, bool blocked);
    void setHorizontalWall(int i, int j, bool blocked);
    void addBoxWall(int i0, int j0, int i1, int j1);

    // Mark a cell occupied by a solid object. For moving objects, solidU/solidV
    // is the object's surface velocity at that cell centre; projection uses it
    // as the boundary velocity, so the object pushes the fluid.
    void setSolidCell(int i, int j, bool solid, float solidU = 0.0f, float solidV = 0.0f);
    bool isSolidCell(int i, int j) const;
    bool isBlocked(int i0, int j0, int i1, int j1) const;

    // Advance density and velocity by one Stable Fluids time step.
    void step(float dt);

    float density(int i, int j) const { return m_density[ix(i, j)]; }
    float temperature(int i, int j) const { return m_temperature[ix(i, j)]; }
    float velocityU(int i, int j) const { return m_u[ix(i, j)]; }
    float velocityV(int i, int j) const { return m_v[ix(i, j)]; }
    // Diagnostics exposed for the multi-field visualization. curl(i,j) is the
    // scalar 2D vorticity (dv/dx - du/dy); pressure(i,j) is the field solved in
    // the last projection step.
    float curl(int i, int j) const;
    float pressure(int i, int j) const;
    Vec2f sampleVelocity(float x, float y) const;
    float sampleDensity(float x, float y) const;
    float sampleTemperature(float x, float y) const;

    const std::vector<float>& densityField() const { return m_density; }
    const std::vector<float>& temperatureField() const { return m_temperature; }
    const std::vector<unsigned char>& solidField() const { return m_solid; }

    // Timing instrumentation (used for the report's per-step timings). step()
    // records its own wall-clock cost; avgStepMs() is an exponential moving
    // average so the value settles a couple of seconds after a feature toggle.
    double lastStepMs() const { return m_lastStepMs; }
    double avgStepMs() const { return m_avgStepMs; }
    void resetTiming() { m_lastStepMs = 0.0; m_avgStepMs = 0.0; m_timingSamples = 0; }

private:
    int m_N;
    float m_diffusion;
    float m_viscosity;
    float m_vorticityStrength;
    bool m_useVorticity;
    float m_temperatureDiffusion;
    float m_ambientTemperature;
    float m_buoyancyAlpha;
    float m_buoyancyBeta;
    bool m_useBuoyancy;

    std::vector<float> m_u, m_v, m_uPrev, m_vPrev;
    std::vector<float> m_density, m_densityPrev;
    std::vector<float> m_temperature, m_temperaturePrev;
    std::vector<float> m_pressure, m_divergence;
    std::vector<float> m_solidU, m_solidV;
    std::vector<unsigned char> m_solid;
    std::vector<unsigned char> m_wallX, m_wallY;

    double m_lastStepMs = 0.0;
    double m_avgStepMs = 0.0;
    long m_timingSamples = 0;

    void addSource(std::vector<float>& x, const std::vector<float>& s, float dt);
    void setBoundary(int b, std::vector<float>& x);
    void linearSolve(int b, std::vector<float>& x, const std::vector<float>& x0, float a, float c);
    void diffuse(int b, std::vector<float>& x, const std::vector<float>& x0, float diff, float dt);
    void advect(int b, std::vector<float>& d, const std::vector<float>& d0,
                const std::vector<float>& u, const std::vector<float>& v, float dt);
    void project(std::vector<float>& u, std::vector<float>& v, std::vector<float>& p, std::vector<float>& div);
    // Fedkiw/Stam/Jensen vorticity confinement: compute scalar curl omega,
    // normalize grad(|omega|), then add epsilon*(N x omega) as a body force.
    void applyVorticityConfinement(float dt);
    // Thermal buoyancy body force (Fedkiw/Stam/Jensen 2001, eq. for f_buoy).
    // Adds (-alpha*density + beta*(T - Tambient)) to the upward velocity.
    void applyBuoyancy(float dt);
    void applySolidVelocities();
    void rebuildObjectWalls();
    void addOuterWalls();

    float bilinearSample(const std::vector<float>& field, float x, float y) const;
    // Semi-Lagrangian backtrace with solid-boundary clipping.
    // This is the assignment's path-clipping requirement for advection near solid walls.
    void traceBackWithClipping(int i, int j, float& x, float& y, const std::vector<float>& u, const std::vector<float>& v, float dt) const;
    float valueForNeighbor(const std::vector<float>& field, int i, int j, int b) const;
};
