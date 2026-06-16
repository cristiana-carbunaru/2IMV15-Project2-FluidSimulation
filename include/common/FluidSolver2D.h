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

    // Temperature/buoyancy
    // The temperature field is transported just like density; the buoyancy force
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

    void setVerticalWall(int i, int j, bool blocked);
    void setHorizontalWall(int i, int j, bool blocked);
    void addBoxWall(int i0, int j0, int i1, int j1);

    // Mark a cell occupied by a solid object. For moving objects, solidU/solidV
    // is the object's surface velocity at that cell centre; projection uses it
    // as the boundary velocity, so the object pushes the fluid.
    void setSolidCell(int i, int j, bool solid, float solidU = 0.0f, float solidV = 0.0f);
    void setSolidVelocity(int i, int j, float solidU, float solidV);
    bool isSolidCell(int i, int j) const;
    bool isBlocked(int i0, int j0, int i1, int j1) const;


    // Advance density and velocity by one Stable Fluids time step.
    void step(float dt);

    float density(int i, int j) const { return m_density[ix(i, j)]; }
    float temperature(int i, int j) const { return m_temperature[ix(i, j)]; }
    float velocityU(int i, int j) const { return m_u[ix(i, j)]; }
    float velocityV(int i, int j) const { return m_v[ix(i, j)]; }
    // Diagnostics exposed for the multi-field visualization. 
    float curl(int i, int j) const;
    float pressure(int i, int j) const;
    Vec2f sampleVelocity(float x, float y) const;
    float sampleDensity(float x, float y) const;
    float sampleTemperature(float x, float y) const;

    const std::vector<float>& densityField() const { return m_density; }
    const std::vector<float>& temperatureField() const { return m_temperature; }
    const std::vector<unsigned char>& solidField() const { return m_solid; }
    
    void initWater(float fillRatio = 0.33f);
    void clearWater();
    const std::vector<Vec2f>& waterParticles() const { return m_waterParticles; }

    // Timing instrumentation step(), avgStepMs() 
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
    
    std::vector<Vec2f> m_waterParticles;
    std::vector<bool> m_isWater;

    double m_lastStepMs = 0.0;
    double m_avgStepMs = 0.0;
    long m_timingSamples = 0;

    void addSource(std::vector<float>& x, const std::vector<float>& s, float dt);
    void setBoundary(int b, std::vector<float>& x);
    void linearSolve(int b, std::vector<float>& x, const std::vector<float>& x0, float a);
    void diffuse(int b, std::vector<float>& x, const std::vector<float>& x0, float diff, float dt);
    void advect(int b, std::vector<float>& d, const std::vector<float>& d0,
                const std::vector<float>& u, const std::vector<float>& v, float dt);
    void project(std::vector<float>& u, std::vector<float>& v, std::vector<float>& p, std::vector<float>& div);
    // Fedkiw/Stam/Jensen vorticity confinement: compute scalar curl omega,
    // normalize grad(|omega|), then add epsilon*(N x omega) as a body force.
    void applyVorticityConfinement(float dt);
    // Thermal buoyancy body force 
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
