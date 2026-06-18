#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <gfx/vec2.h>

class FluidSolver2D {
public:
    FluidSolver2D();
    explicit FluidSolver2D(int n);

    void resize(int n);
    void clear();
    void clearSources();

    void clearSolids();

    int gridSize() const { return m_N; }
    int ix(int i, int j) const { return i + (m_N + 2) * j; }
    int size() const { return (m_N + 2) * (m_N + 2); }

    void setDiffusion(float value) { m_diffusion = std::max(0.0f, value); }
    void setViscosity(float value) { m_viscosity = std::max(0.0f, value); }
    void setVorticityStrength(float value) { m_vorticityStrength = std::max(0.0f, value); }
    void enableVorticity(bool enabled) { m_useVorticity = enabled; }
    bool vorticityEnabled() const { return m_useVorticity; }

    void setTemperatureDiffusion(float value) { m_temperatureDiffusion = std::max(0.0f, value); }
    void setAmbientTemperature(float value) { m_ambientTemperature = value; }
    void setBuoyancy(float alpha, float beta) { m_buoyancyAlpha = std::max(0.0f, alpha); m_buoyancyBeta = std::max(0.0f, beta); }
    void enableBuoyancy(bool enabled) { m_useBuoyancy = enabled; }
    bool buoyancyEnabled() const { return m_useBuoyancy; }

    void addDensityCell(int i, int j, float amount);
    void addVelocityCell(int i, int j, float amountU, float amountV);
    void addDensityAt(float x, float y, float amount, int radius = 1);
    void addVelocityAt(float x, float y, float amountU, float amountV, int radius = 1);

    void addTemperatureCell(int i, int j, float amount);
    void addTemperatureAt(float x, float y, float amount, int radius = 1);

    void setVerticalWall(int i, int j, bool blocked);
    void setHorizontalWall(int i, int j, bool blocked);
    void addBoxWall(int i0, int j0, int i1, int j1);

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
    void applyVorticityConfinement(float dt);
    // Thermal buoyancy body force 
    void applyBuoyancy(float dt);
    void applySolidVelocities();
    void rebuildObjectWalls();
    void addOuterWalls();

    float bilinearSample(const std::vector<float>& field, float x, float y) const;
    void traceBackWithClipping(int i, int j, float& x, float& y, const std::vector<float>& u, const std::vector<float>& v, float dt) const;
    float valueForNeighbor(const std::vector<float>& field, int i, int j, int b) const;
};
