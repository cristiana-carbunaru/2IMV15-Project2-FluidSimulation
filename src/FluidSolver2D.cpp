// FluidSolver2D.cpp:
// Numerical core of the Project 2 fluid simulation. The solver follows the
// Stable Fluids algorithm (Stam 1999 / Stam GDC 2003) and then adds the extra
// assignment features in local, commented methods. The comments deliberately
// explain the equations in code terms so that the implementation can be read
// together with the report/notes.

#include "FluidSolver2D.h"
#include <cstdio>
#include <chrono>

#define FLUID_SWAP(a,b) { std::swap(a,b); }

FluidSolver2D::FluidSolver2D()
    : m_N(0), m_diffusion(0.0f), m_viscosity(0.0f), m_vorticityStrength(2.0f), m_useVorticity(true),
      m_temperatureDiffusion(0.00002f), m_ambientTemperature(0.0f),
      m_buoyancyAlpha(0.05f), m_buoyancyBeta(0.3f), m_useBuoyancy(true) {}

FluidSolver2D::FluidSolver2D(int n) : FluidSolver2D() { resize(n); }

// Allocate all fields. The +2 ghost-cell border follows Stam's original code;
// physical cells are 1..N, and indices 0 and N+1 store boundary values.
void FluidSolver2D::resize(int n) {
    m_N = std::max(4, n);
    const int s = size();
    m_u.assign(s, 0.0f);      m_v.assign(s, 0.0f);
    m_uPrev.assign(s, 0.0f);  m_vPrev.assign(s, 0.0f);
    m_density.assign(s, 0.0f); m_densityPrev.assign(s, 0.0f);
    m_temperature.assign(s, m_ambientTemperature); m_temperaturePrev.assign(s, 0.0f);
    m_pressure.assign(s, 0.0f); m_divergence.assign(s, 0.0f);
    m_solidU.assign(s, 0.0f); m_solidV.assign(s, 0.0f);
    m_solid.assign(s, 0);
    m_wallX.assign(s, 0);
    m_wallY.assign(s, 0);
    addOuterWalls();
}

void FluidSolver2D::clear() {
    std::fill(m_u.begin(), m_u.end(), 0.0f);
    std::fill(m_v.begin(), m_v.end(), 0.0f);
    std::fill(m_uPrev.begin(), m_uPrev.end(), 0.0f);
    std::fill(m_vPrev.begin(), m_vPrev.end(), 0.0f);
    std::fill(m_density.begin(), m_density.end(), 0.0f);
    std::fill(m_densityPrev.begin(), m_densityPrev.end(), 0.0f);
    std::fill(m_temperature.begin(), m_temperature.end(), m_ambientTemperature);
    std::fill(m_temperaturePrev.begin(), m_temperaturePrev.end(), 0.0f);
    std::fill(m_pressure.begin(), m_pressure.end(), 0.0f);
    std::fill(m_divergence.begin(), m_divergence.end(), 0.0f);
}

void FluidSolver2D::clearSources() {
    std::fill(m_uPrev.begin(), m_uPrev.end(), 0.0f);
    std::fill(m_vPrev.begin(), m_vPrev.end(), 0.0f);
    std::fill(m_densityPrev.begin(), m_densityPrev.end(), 0.0f);
    std::fill(m_temperaturePrev.begin(), m_temperaturePrev.end(), 0.0f);
}

void FluidSolver2D::clearSolids() {
    std::fill(m_solid.begin(), m_solid.end(), 0);
    std::fill(m_solidU.begin(), m_solidU.end(), 0.0f);
    std::fill(m_solidV.begin(), m_solidV.end(), 0.0f);
    std::fill(m_wallX.begin(), m_wallX.end(), 0);
    std::fill(m_wallY.begin(), m_wallY.end(), 0);
    addOuterWalls();
}

// The assignment states that the outside of the grid is solid. We represent
// this exactly the same way as internal edge walls: blocked vertical/horizontal
// edges around the active 1..N cells.
void FluidSolver2D::addOuterWalls() {
    for (int j = 1; j <= m_N; ++j) {
        m_wallX[ix(1, j)] = 1;
        m_wallX[ix(m_N + 1, j)] = 1;
    }
    for (int i = 1; i <= m_N; ++i) {
        m_wallY[ix(i, 1)] = 1;
        m_wallY[ix(i, m_N + 1)] = 1;
    }
}

void FluidSolver2D::addDensityCell(int i, int j, float amount) {
    if (i < 1 || i > m_N || j < 1 || j > m_N || isSolidCell(i, j)) return;
    m_densityPrev[ix(i, j)] += amount;
}

void FluidSolver2D::addVelocityCell(int i, int j, float amountU, float amountV) {
    if (i < 1 || i > m_N || j < 1 || j > m_N || isSolidCell(i, j)) return;
    m_uPrev[ix(i, j)] += amountU;
    m_vPrev[ix(i, j)] += amountV;
}

void FluidSolver2D::addDensityAt(float x, float y, float amount, int radius) {
    int ci = std::max(1, std::min(m_N, int(x * m_N) + 1));
    int cj = std::max(1, std::min(m_N, int(y * m_N) + 1));
    for (int j = cj - radius; j <= cj + radius; ++j)
        for (int i = ci - radius; i <= ci + radius; ++i)
            addDensityCell(i, j, amount);
}

void FluidSolver2D::addVelocityAt(float x, float y, float amountU, float amountV, int radius) {
    int ci = std::max(1, std::min(m_N, int(x * m_N) + 1));
    int cj = std::max(1, std::min(m_N, int(y * m_N) + 1));
    for (int j = cj - radius; j <= cj + radius; ++j)
        for (int i = ci - radius; i <= ci + radius; ++i)
            addVelocityCell(i, j, amountU, amountV);
}

void FluidSolver2D::addTemperatureCell(int i, int j, float amount) {
    if (i < 1 || i > m_N || j < 1 || j > m_N || isSolidCell(i, j)) return;
    m_temperaturePrev[ix(i, j)] += amount;
}

void FluidSolver2D::addTemperatureAt(float x, float y, float amount, int radius) {
    int ci = std::max(1, std::min(m_N, int(x * m_N) + 1));
    int cj = std::max(1, std::min(m_N, int(y * m_N) + 1));
    for (int j = cj - radius; j <= cj + radius; ++j)
        for (int i = ci - radius; i <= ci + radius; ++i)
            addTemperatureCell(i, j, amount);
}

void FluidSolver2D::setVerticalWall(int i, int j, bool blocked) {
    if (i < 1 || i > m_N + 1 || j < 1 || j > m_N) return;
    m_wallX[ix(i, j)] = blocked ? 1 : 0;
}

void FluidSolver2D::setHorizontalWall(int i, int j, bool blocked) {
    if (i < 1 || i > m_N || j < 1 || j > m_N + 1) return;
    m_wallY[ix(i, j)] = blocked ? 1 : 0;
}

// Convenience helper for a fixed rectangular obstacle made only from boundary
// edges. Notice that it does not fill the interior with solid cells; it proves
// that the solver can mark arbitrary neighbour-cell edges as boundaries.
void FluidSolver2D::addBoxWall(int i0, int j0, int i1, int j1) {
    if (i0 > i1) std::swap(i0, i1);
    if (j0 > j1) std::swap(j0, j1);
    i0 = std::max(1, std::min(m_N, i0));
    i1 = std::max(1, std::min(m_N, i1));
    j0 = std::max(1, std::min(m_N, j0));
    j1 = std::max(1, std::min(m_N, j1));
    for (int j = j0; j <= j1; ++j) {
        setVerticalWall(i0, j, true);
        setVerticalWall(i1 + 1, j, true);
    }
    for (int i = i0; i <= i1; ++i) {
        setHorizontalWall(i, j0, true);
        setHorizontalWall(i, j1 + 1, true);
    }
}

void FluidSolver2D::setSolidCell(int i, int j, bool solid, float solidU, float solidV) {
    if (i < 1 || i > m_N || j < 1 || j > m_N) return;
    int k = ix(i, j);
    m_solid[k] = solid ? 1 : 0;
    m_solidU[k] = solid ? solidU : 0.0f;
    m_solidV[k] = solid ? solidV : 0.0f;
    if (solid) {
        m_density[k] = 0.0f;
        m_densityPrev[k] = 0.0f;
        m_u[k] = solidU;
        m_v[k] = solidV;
    }
}

void FluidSolver2D::setSolidVelocity(int i, int j, float solidU, float solidV) {
    if (i < 1 || i > m_N || j < 1 || j > m_N) return;
    int k = ix(i, j);
    if (m_solid[k]) {
        m_solidU[k] = solidU;
        m_solidV[k] = solidV;
        m_u[k] = solidU;
        m_v[k] = solidV;
    }
}

bool FluidSolver2D::isSolidCell(int i, int j) const {
    if (i < 1 || i > m_N || j < 1 || j > m_N) return true;
    return m_solid[ix(i, j)] != 0;
}

// Test whether information may pass between two neighbouring cells. This method
// is used consistently by diffusion, projection, and advection-path clipping.
bool FluidSolver2D::isBlocked(int i0, int j0, int i1, int j1) const {
    if (i1 < 1 || i1 > m_N || j1 < 1 || j1 > m_N) return true;
    if (i0 < 1 || i0 > m_N || j0 < 1 || j0 > m_N) return true;
    if (isSolidCell(i0, j0) || isSolidCell(i1, j1)) return true;
    if (i0 != i1) {
        const int ei = std::max(i0, i1);
        return m_wallX[ix(ei, j0)] != 0;
    }
    if (j0 != j1) {
        const int ej = std::max(j0, j1);
        return m_wallY[ix(i0, ej)] != 0;
    }
    return false;
}

void FluidSolver2D::rebuildObjectWalls() {
    // Existing user-created walls remain; solid objects add cell-aligned impermeable edges.
    for (int j = 1; j <= m_N; ++j) {
        for (int i = 1; i <= m_N; ++i) {
            if (!isSolidCell(i, j)) continue;
            if (i > 1) setVerticalWall(i, j, true);
            if (i <= m_N) setVerticalWall(i + 1, j, true);
            if (j > 1) setHorizontalWall(i, j, true);
            if (j <= m_N) setHorizontalWall(i, j + 1, true);
        }
    }
    addOuterWalls();
}

// Stam's explicit source addition: x += dt*s. Mouse forces, density sources,
// and temporary body impulses are placed in the source arrays before stepping.
void FluidSolver2D::addSource(std::vector<float>& x, const std::vector<float>& s, float dt) {
    for (int i = 0; i < size(); ++i) x[i] += dt * s[i];
}

float FluidSolver2D::valueForNeighbor(const std::vector<float>& field, int i, int j, int b) const {
    if (i < 1 || i > m_N || j < 1 || j > m_N) return 0.0f;
    if (!isSolidCell(i, j)) return field[ix(i, j)];
    if (b == 1) return m_solidU[ix(i, j)];
    if (b == 2) return m_solidV[ix(i, j)];
    return 0.0f;
}

// Apply boundary conditions. Parameter b follows Stam's convention:
//   b=0 scalar field, b=1 horizontal velocity, b=2 vertical velocity.
// At outer walls, the normal velocity component is reflected. At moving solid
// cells, the velocity is set to the object's prescribed surface velocity.
void FluidSolver2D::setBoundary(int b, std::vector<float>& x) {
    for (int i = 1; i <= m_N; ++i) {
        x[ix(0, i)] = b == 1 ? -x[ix(1, i)] : x[ix(1, i)];
        x[ix(m_N + 1, i)] = b == 1 ? -x[ix(m_N, i)] : x[ix(m_N, i)];
        x[ix(i, 0)] = b == 2 ? -x[ix(i, 1)] : x[ix(i, 1)];
        x[ix(i, m_N + 1)] = b == 2 ? -x[ix(i, m_N)] : x[ix(i, m_N)];
    }
    x[ix(0, 0)] = 0.5f * (x[ix(1, 0)] + x[ix(0, 1)]);
    x[ix(0, m_N + 1)] = 0.5f * (x[ix(1, m_N + 1)] + x[ix(0, m_N)]);
    x[ix(m_N + 1, 0)] = 0.5f * (x[ix(m_N, 0)] + x[ix(m_N + 1, 1)]);
    x[ix(m_N + 1, m_N + 1)] = 0.5f * (x[ix(m_N, m_N + 1)] + x[ix(m_N + 1, m_N)]);

    for (int j = 1; j <= m_N; ++j) {
        for (int i = 1; i <= m_N; ++i) {
            const int k = ix(i, j);
            if (m_solid[k]) {
                if (b == 1) x[k] = m_solidU[k];
                else if (b == 2) x[k] = m_solidV[k];
                else x[k] = 0.0f;
                continue;
            }
            if (b == 1) {
                if ((i > 1 && isBlocked(i, j, i - 1, j)) || (i < m_N && isBlocked(i, j, i + 1, j))) {
                    // Collocated approximation: match the nearest solid face velocity for no-through motion.
                    if (i > 1 && isSolidCell(i - 1, j)) x[k] = m_solidU[ix(i - 1, j)];
                    if (i < m_N && isSolidCell(i + 1, j)) x[k] = m_solidU[ix(i + 1, j)];
                }
            } else if (b == 2) {
                if ((j > 1 && isBlocked(i, j, i, j - 1)) || (j < m_N && isBlocked(i, j, i, j + 1))) {
                    if (j > 1 && isSolidCell(i, j - 1)) x[k] = m_solidV[ix(i, j - 1)];
                    if (j < m_N && isSolidCell(i, j + 1)) x[k] = m_solidV[ix(i, j + 1)];
                }
            }
        }
    }
}

// Gauss-Seidel relaxation for both diffusion and pressure-like solves.
// Standard Stam denominator is c = 1 + 4a. When a cell has blocked neighbours,
// only unblocked neighbours are included; the denominator is adjusted so walls
// behave like zero-flux/no-through boundaries.
void FluidSolver2D::linearSolve(int b, std::vector<float>& x, const std::vector<float>& x0, float a) {
    for (int k = 0; k < 25; ++k) {
        for (int j = 1; j <= m_N; ++j) {
            for (int i = 1; i <= m_N; ++i) {
                if (isSolidCell(i, j)) continue;
                float sum = 0.0f;
                int count = 0;
                if (!isBlocked(i, j, i - 1, j)) { sum += x[ix(i - 1, j)]; ++count; }
                if (!isBlocked(i, j, i + 1, j)) { sum += x[ix(i + 1, j)]; ++count; }
                if (!isBlocked(i, j, i, j - 1)) { sum += x[ix(i, j - 1)]; ++count; }
                if (!isBlocked(i, j, i, j + 1)) { sum += x[ix(i, j + 1)]; ++count; }
                x[ix(i, j)] = (x0[ix(i, j)] + a * sum) / (1.0f + a * count);
            }
        }
        setBoundary(b, x);
    }
}

// Implicit diffusion: solve x - a*Laplacian(x) = x0. This is the Stable Fluids
// reason diffusion stays stable for interactive time steps.
void FluidSolver2D::diffuse(int b, std::vector<float>& x, const std::vector<float>& x0, float diff, float dt) {
    const float a = dt * diff * m_N * m_N;
    linearSolve(b, x, x0, a);
}

// Semi-Lagrangian backtrace from the centre of cell (i,j). We subdivide the
// segment to detect crossing a blocked edge. If a wall is hit, sampling stops on
// the fluid side of the wall instead of sampling through the solid.
void FluidSolver2D::traceBackWithClipping(int i, int j, float& x, float& y,
                                          const std::vector<float>& u, const std::vector<float>& v, float dt) const {
    const float dt0 = dt * m_N;
    const float startX = static_cast<float>(i);
    const float startY = static_cast<float>(j);
    const float endX = std::max(0.5f, std::min(m_N + 0.5f, startX - dt0 * u[ix(i, j)]));
    const float endY = std::max(0.5f, std::min(m_N + 0.5f, startY - dt0 * v[ix(i, j)]));

    int ci = i, cj = j;
    float px = startX, py = startY;
    const int steps = std::max(4, static_cast<int>(std::ceil(std::sqrt((endX - startX) * (endX - startX) + (endY - startY) * (endY - startY)))) * 2);
    for (int s = 1; s <= steps; ++s) {
        const float t = static_cast<float>(s) / steps;
        const float nx = startX + (endX - startX) * t;
        const float ny = startY + (endY - startY) * t;
        const int ni = std::max(1, std::min(m_N, static_cast<int>(std::floor(nx + 0.5f))));
        const int nj = std::max(1, std::min(m_N, static_cast<int>(std::floor(ny + 0.5f))));
        if ((ni != ci || nj != cj) && isBlocked(ci, cj, ni, nj)) {
            x = px;
            y = py;
            return;
        }
        px = nx; py = ny; ci = ni; cj = nj;
    }
    x = endX; y = endY;
}

// Bilinear interpolation used by advection and by particles/cloth. Solid cells
// contribute zero for scalar sampling; velocity boundary values are handled by
// setBoundary/project before sampling.
float FluidSolver2D::bilinearSample(const std::vector<float>& field, float x, float y) const {
    x = std::max(0.5f, std::min(m_N + 0.5f, x));
    y = std::max(0.5f, std::min(m_N + 0.5f, y));
    int i0 = static_cast<int>(x); int i1 = i0 + 1;
    int j0 = static_cast<int>(y); int j1 = j0 + 1;
    i0 = std::max(1, std::min(m_N, i0)); i1 = std::max(1, std::min(m_N, i1));
    j0 = std::max(1, std::min(m_N, j0)); j1 = std::max(1, std::min(m_N, j1));
    const float s1 = x - static_cast<int>(x);
    const float s0 = 1.0f - s1;
    const float t1 = y - static_cast<int>(y);
    const float t0 = 1.0f - t1;
    auto val = [&](int ii, int jj) -> float {
        if (isSolidCell(ii, jj)) return 0.0f;
        return field[ix(ii, jj)];
    };
    return s0 * (t0 * val(i0, j0) + t1 * val(i0, j1)) +
           s1 * (t0 * val(i1, j0) + t1 * val(i1, j1));
}

// Semi-Lagrangian advection. For each destination cell, ask where the material
// came from at the previous time step, then interpolate the old field there.
// This mirrors Stam's advect() but uses traceBackWithClipping for boundaries.
void FluidSolver2D::advect(int b, std::vector<float>& d, const std::vector<float>& d0,
                           const std::vector<float>& u, const std::vector<float>& v, float dt) {
    for (int j = 1; j <= m_N; ++j) {
        for (int i = 1; i <= m_N; ++i) {
            if (isSolidCell(i, j)) {
                d[ix(i, j)] = (b == 1) ? m_solidU[ix(i, j)] : ((b == 2) ? m_solidV[ix(i, j)] : 0.0f);
                continue;
            }
            float x, y;
            traceBackWithClipping(i, j, x, y, u, v, dt);
            d[ix(i, j)] = bilinearSample(d0, x, y);
        }
    }
    setBoundary(b, d);
}

// Pressure projection. Compute divergence, solve a Poisson equation for p, then
// subtract grad(p). The result is approximately divergence-free. Moving solids
// enter here through valueForNeighbor(), which supplies solid surface velocity
// when a fluid cell neighbours an occupied cell.
void FluidSolver2D::project(std::vector<float>& u, std::vector<float>& v, std::vector<float>& p, std::vector<float>& div) {
    for (int j = 1; j <= m_N; ++j) {
        for (int i = 1; i <= m_N; ++i) {
            const int k = ix(i, j);
            p[k] = 0.0f;
            if (isSolidCell(i, j)) { div[k] = 0.0f; continue; }

            const float uR = isBlocked(i, j, i + 1, j) ? valueForNeighbor(u, i + 1, j, 1) : u[ix(i + 1, j)];
            const float uL = isBlocked(i, j, i - 1, j) ? valueForNeighbor(u, i - 1, j, 1) : u[ix(i - 1, j)];
            const float vT = isBlocked(i, j, i, j + 1) ? valueForNeighbor(v, i, j + 1, 2) : v[ix(i, j + 1)];
            const float vB = isBlocked(i, j, i, j - 1) ? valueForNeighbor(v, i, j - 1, 2) : v[ix(i, j - 1)];
            div[k] = -0.5f * (uR - uL + vT - vB) / m_N;
        }
    }
    setBoundary(0, div);
    setBoundary(0, p);

    for (int iter = 0; iter < 40; ++iter) {
        for (int j = 1; j <= m_N; ++j) {
            for (int i = 1; i <= m_N; ++i) {
                if (isSolidCell(i, j)) continue;
                float sum = 0.0f;
                int count = 0;
                if (!isBlocked(i, j, i - 1, j)) { sum += p[ix(i - 1, j)]; ++count; }
                if (!isBlocked(i, j, i + 1, j)) { sum += p[ix(i + 1, j)]; ++count; }
                if (!isBlocked(i, j, i, j - 1)) { sum += p[ix(i, j - 1)]; ++count; }
                if (!isBlocked(i, j, i, j + 1)) { sum += p[ix(i, j + 1)]; ++count; }
                p[ix(i, j)] = count > 0 ? (div[ix(i, j)] + sum) / static_cast<float>(count) : 0.0f;
            }
        }
        setBoundary(0, p);
    }

    for (int j = 1; j <= m_N; ++j) {
        for (int i = 1; i <= m_N; ++i) {
            if (isSolidCell(i, j)) continue;
            if (!isBlocked(i, j, i + 1, j) && !isBlocked(i, j, i - 1, j))
                u[ix(i, j)] -= 0.5f * m_N * (p[ix(i + 1, j)] - p[ix(i - 1, j)]);
            else if (isBlocked(i, j, i + 1, j))
                u[ix(i, j)] -= 0.5f * m_N * (p[ix(i, j)] - p[ix(i - 1, j)]);
            else if (isBlocked(i, j, i - 1, j))
                u[ix(i, j)] -= 0.5f * m_N * (p[ix(i + 1, j)] - p[ix(i, j)]);

            if (!isBlocked(i, j, i, j + 1) && !isBlocked(i, j, i, j - 1))
                v[ix(i, j)] -= 0.5f * m_N * (p[ix(i, j + 1)] - p[ix(i, j - 1)]);
            else if (isBlocked(i, j, i, j + 1))
                v[ix(i, j)] -= 0.5f * m_N * (p[ix(i, j)] - p[ix(i, j - 1)]);
            else if (isBlocked(i, j, i, j - 1))
                v[ix(i, j)] -= 0.5f * m_N * (p[ix(i, j + 1)] - p[ix(i, j)]);
        }
    }
    setBoundary(1, u);
    setBoundary(2, v);
    m_pressure = p;
    applySolidVelocities();
}

// Scalar 2D vorticity at a cell centre. Reused by the curl visualization view;
// the same expression drives applyVorticityConfinement.
float FluidSolver2D::curl(int i, int j) const {
    if (i < 1 || i > m_N || j < 1 || j > m_N) return 0.0f;
    const float dvdx = 0.5f * (m_v[ix(i + 1, j)] - m_v[ix(i - 1, j)]);
    const float dudy = 0.5f * (m_u[ix(i, j + 1)] - m_u[ix(i, j - 1)]);
    return dvdx - dudy;
}

float FluidSolver2D::pressure(int i, int j) const {
    if (i < 1 || i > m_N || j < 1 || j > m_N) return 0.0f;
    return m_pressure[ix(i, j)];
}

// Vorticity confinement from Fedkiw, Stam, and Jensen (2001). In 2D the curl is
// scalar: omega = dv/dx - du/dy. We compute N = grad(|omega|)/|grad(|omega|)|
// and add epsilon*(N x omega). Since omega points out of the screen,
// N x omega = (Ny*omega, -Nx*omega). The paper includes an h factor; here the
// tunable m_vorticityStrength absorbs that scale for the demo grid.
void FluidSolver2D::applyVorticityConfinement(float dt) {
    if (!m_useVorticity || m_vorticityStrength <= 0.0f) return;
    std::vector<float> curl(size(), 0.0f);
    for (int j = 2; j <= m_N - 1; ++j) {
        for (int i = 2; i <= m_N - 1; ++i) {
            if (isSolidCell(i, j)) continue;
            const float dvdx = 0.5f * (m_v[ix(i + 1, j)] - m_v[ix(i - 1, j)]);
            const float dudy = 0.5f * (m_u[ix(i, j + 1)] - m_u[ix(i, j - 1)]);
            curl[ix(i, j)] = dvdx - dudy;
        }
    }
    for (int j = 2; j <= m_N - 1; ++j) {
        for (int i = 2; i <= m_N - 1; ++i) {
            if (isSolidCell(i, j)) continue;
            const float gradX = 0.5f * (std::fabs(curl[ix(i + 1, j)]) - std::fabs(curl[ix(i - 1, j)]));
            const float gradY = 0.5f * (std::fabs(curl[ix(i, j + 1)]) - std::fabs(curl[ix(i, j - 1)]));
            const float len = std::sqrt(gradX * gradX + gradY * gradY) + 1e-6f;
            const float Nx = gradX / len;
            const float Ny = gradY / len;
            const float w = curl[ix(i, j)];
            m_u[ix(i, j)] += dt * m_vorticityStrength * (Ny * w);
            m_v[ix(i, j)] += dt * m_vorticityStrength * (-Nx * w);
        }
    }
    setBoundary(1, m_u);
    setBoundary(2, m_v);
}

// Thermal buoyancy (Fedkiw, Stam, and Jensen 2001, "Visual Simulation of Smoke").
// They define a buoyancy force f_buoy = (-alpha*d + beta*(T - T_ambient)) * z,
// where z points up. Here z is the +v (vertical) direction of the grid, so the
// dense-smoke term -alpha*d pulls fluid down while the warm term beta*(T-Tamb)
// pushes it up, making hot air rise. The force is added to the velocity field
// before the diffuse/project velocity solve, exactly like the vorticity force.
void FluidSolver2D::applyBuoyancy(float dt) {
    if (!m_useBuoyancy) return;
    if (m_buoyancyAlpha <= 0.0f && m_buoyancyBeta <= 0.0f) return;
    for (int j = 1; j <= m_N; ++j) {
        for (int i = 1; i <= m_N; ++i) {
            if (isSolidCell(i, j)) continue;
            const int k = ix(i, j);
            const float f = -m_buoyancyAlpha * m_density[k]
                            + m_buoyancyBeta * (m_temperature[k] - m_ambientTemperature);
            m_v[k] += dt * f;
        }
    }
    setBoundary(2, m_v);
}

// Keep occupied solid cells from accumulating smoke or arbitrary fluid velocity.
// Their velocity is prescribed by the rigid body rasterization in FluidScene.
void FluidSolver2D::applySolidVelocities() {
    for (int j = 1; j <= m_N; ++j) {
        for (int i = 1; i <= m_N; ++i) {
            const int k = ix(i, j);
            if (m_solid[k]) {
                m_u[k] = m_solidU[k];
                m_v[k] = m_solidV[k];
                m_density[k] = 0.0f;
            }
        }
    }
}

// Complete Stable Fluids update. The order is intentionally close to Stam's
// vel_step()/dens_step(), with vorticity and solid velocity insertion before
// the velocity solve so those forces affect the current frame.
void FluidSolver2D::step(float dt) {
    const auto t0 = std::chrono::high_resolution_clock::now();
    rebuildObjectWalls();
    addSource(m_u, m_uPrev, dt);
    addSource(m_v, m_vPrev, dt);
    // Inject heat sources into the temperature field before it drives buoyancy.
    addSource(m_temperature, m_temperaturePrev, dt);
    applySolidVelocities();
    applyVorticityConfinement(dt);
    applyBuoyancy(dt);

    FLUID_SWAP(m_uPrev, m_u);
    diffuse(1, m_u, m_uPrev, m_viscosity, dt);
    FLUID_SWAP(m_vPrev, m_v);
    diffuse(2, m_v, m_vPrev, m_viscosity, dt);
    project(m_u, m_v, m_uPrev, m_vPrev);

    FLUID_SWAP(m_uPrev, m_u);
    FLUID_SWAP(m_vPrev, m_v);
    advect(1, m_u, m_uPrev, m_uPrev, m_vPrev, dt);
    advect(2, m_v, m_vPrev, m_uPrev, m_vPrev, dt);
    project(m_u, m_v, m_uPrev, m_vPrev);

    addSource(m_density, m_densityPrev, dt);
    FLUID_SWAP(m_densityPrev, m_density);
    diffuse(0, m_density, m_densityPrev, m_diffusion, dt);
    FLUID_SWAP(m_densityPrev, m_density);
    advect(0, m_density, m_densityPrev, m_u, m_v, dt);

    // Transport temperature with the same diffuse/advect path as density so the
    // heat that drives buoyancy moves with the flow (the heat source was already
    // added above, before applyBuoyancy).
    FLUID_SWAP(m_temperaturePrev, m_temperature);
    diffuse(0, m_temperature, m_temperaturePrev, m_temperatureDiffusion, dt);
    FLUID_SWAP(m_temperaturePrev, m_temperature);
    advect(0, m_temperature, m_temperaturePrev, m_u, m_v, dt);

    // Gentle dissipation keeps the real-time demo readable instead of saturating to white.
    for (float& d : m_density) d *= 0.997f;
    // Newtonian cooling: relax temperature back toward ambient so plumes lose lift
    // over time instead of accelerating forever.
    for (float& t : m_temperature) t += (m_ambientTemperature - t) * 0.01f;
    clearSources();

    const auto t1 = std::chrono::high_resolution_clock::now();
    m_lastStepMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    ++m_timingSamples;
    const double alpha = 0.05; // EMA: settles ~60 steps after a feature toggle.
    m_avgStepMs = (m_timingSamples == 1) ? m_lastStepMs
                                         : (1.0 - alpha) * m_avgStepMs + alpha * m_lastStepMs;
}

// Sample velocity at normalized coordinates [0,1]^2. Used by tracer particles,
// cloth drag, and approximate two-way coupling from fluid to rigid bodies.
Vec2f FluidSolver2D::sampleVelocity(float x, float y) const {
    const float gx = std::max(0.5f, std::min(m_N + 0.5f, x * m_N + 0.5f));
    const float gy = std::max(0.5f, std::min(m_N + 0.5f, y * m_N + 0.5f));
    return Vec2f(bilinearSample(m_u, gx, gy), bilinearSample(m_v, gx, gy));
}

float FluidSolver2D::sampleDensity(float x, float y) const {
    const float gx = std::max(0.5f, std::min(m_N + 0.5f, x * m_N + 0.5f));
    const float gy = std::max(0.5f, std::min(m_N + 0.5f, y * m_N + 0.5f));
    return bilinearSample(m_density, gx, gy);
}

float FluidSolver2D::sampleTemperature(float x, float y) const {
    const float gx = std::max(0.5f, std::min(m_N + 0.5f, x * m_N + 0.5f));
    const float gy = std::max(0.5f, std::min(m_N + 0.5f, y * m_N + 0.5f));
    return bilinearSample(m_temperature, gx, gy);
}
