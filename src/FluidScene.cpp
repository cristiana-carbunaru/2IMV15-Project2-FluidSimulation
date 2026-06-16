// FluidScene.cpp:
// Interactive wrapper around FluidSolver2D. This file connects the numerical
// methods to the required demo features: toggles, mouse-driven moving solids,
// rotating rigid bodies, collision impulses, two-way coupling, tracers, and
// cloth. The comments in this file explain how each assignment item is exposed
// in the running program.

#include "FluidScene.h"
#include "GLCompat.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <chrono>

static float length2(const Vec2f& v) { return v * v; }
static float length(const Vec2f& v) { return std::sqrt(length2(v)); }
static Vec2f normalizeSafe(const Vec2f& v, const Vec2f& fallback = Vec2f(1.0f, 0.0f)) {
    const float l = length(v);
    return l > 1e-6f ? v / l : fallback;
}
static float cross2D(const Vec2f& a, const Vec2f& b) { return a[0] * b[1] - a[1] * b[0]; }

// Visualization views cycled by the 'v' key. VIEW_DENSITY and VIEW_VELOCITY are
// the original two; the rest are scalar heatmaps drawn by drawScalarField.
enum ViewMode {
    VIEW_DENSITY = 0,
    VIEW_VELOCITY,
    VIEW_CURL,
    VIEW_PRESSURE,
    VIEW_TEMPERATURE,
    VIEW_COUNT
};

static const char* viewName(int m) {
    switch (m) {
    case VIEW_DENSITY:     return "density";
    case VIEW_VELOCITY:    return "velocity";
    case VIEW_CURL:        return "vorticity (curl)";
    case VIEW_PRESSURE:    return "pressure";
    case VIEW_TEMPERATURE: return "temperature";
    default:               return "density";
    }
}

// Map a value to an RGB colour. Signed fields (curl, pressure) use a diverging
// blue(negative) - dark - red(positive) map, non-negative fields (temperature)
// use a warm black-body-style ramp. The input is already normalized to the
// frame's magnitude: [-1,1] for signed fields, [0,1] otherwise.
static void setHeatColor(float t, bool signedField) {
    if (signedField) {
        const float a = std::min(1.0f, std::fabs(t));
        const float r = std::max(0.0f, t);
        const float b = std::max(0.0f, -t);
        glColor3f(r, 0.12f * a, b);
    } else {
        const float r = std::min(1.0f, t * 2.0f);
        const float g = std::min(1.0f, std::max(0.0f, t * 2.0f - 0.6f));
        const float bch = std::min(1.0f, std::max(0.0f, t * 2.0f - 1.3f));
        glColor3f(r, g, bch);
    }
}

FluidScene::FluidScene()
    : m_solver(80), m_viewMode(VIEW_DENSITY), m_enableVorticity(true), m_enableFixedObjects(true),
      m_enableMovingSolids(true), m_enableRigidRotation(true), m_enableRigidCollisions(true),
      m_enableTwoWayCoupling(true), m_enableParticlesAndCloth(true), m_enableBuoyancy(true),
      m_enableGravity(true), m_selectedBody(-1),
      m_leftDown(false), m_rightDown(false), m_lastMouse(0.0f, 0.0f), m_grabOffset(0.0f, 0.0f),
      m_sourceAmount(280.0f), m_forceScale(8.0f), m_heatAmount(80.0f) {}

// Create the default demo. The first body is fixed (blue), while the two other
// bodies are movable/rotating and can be dragged with the mouse.
void FluidScene::init() {
    m_solver.resize(80);
    m_solver.setDiffusion(0.00002f);
    m_solver.setViscosity(0.00001f);
    m_solver.setVorticityStrength(2.0f);
    m_solver.enableVorticity(m_enableVorticity);
    // Thermal buoyancy (optional Temperature feature): alpha weights the
    // smoke-weight downforce, beta weights the warm-air lift.
    m_solver.setBuoyancy(0.05f, 0.3f);
    m_solver.setAmbientTemperature(0.0f);
    m_solver.setTemperatureDiffusion(0.00002f);
    m_solver.enableBuoyancy(m_enableBuoyancy);
    m_solver.clear();

    m_bodies.clear();
    m_bodies.push_back(RigidBody2D::makeBox(Vec2f(0.50f, 0.52f), Vec2f(0.06f, 0.16f), 2.0f, true));
    m_bodies.push_back(RigidBody2D::makeBox(Vec2f(0.30f, 0.55f), Vec2f(0.07f, 0.05f), 1.0f, false));
    m_bodies.push_back(RigidBody2D::makeCircle(Vec2f(0.70f, 0.46f), 0.06f, 0.8f, false));
    m_bodies[1].velocity = Vec2f(0.08f, 0.0f);
    m_bodies[2].velocity = Vec2f(-0.06f, 0.02f);
    m_bodies[2].angularVelocity = 2.0f;

    seedTracers();
    buildCloth();
    rebuildSolids();
    m_solver.initWater(0.33f);

    // A little initial smoke plume and swirl above the water line
    for (int j = 28; j <= 55; ++j) {
        m_solver.addDensityCell(8, j, 150.0f);
        m_solver.addTemperatureCell(8, j, 60.0f);
        m_solver.addVelocityCell(8, j, 8.0f, 3.0f * std::sin(j * 0.2f));
    }
}

void FluidScene::clear() {
    m_solver.clear();
    seedTracers();
    buildCloth();
    m_solver.initWater(0.33f);
}

// Passive tracer particles: they do not affect the fluid, but they make the
// velocity field visible as moving green points.
void FluidScene::seedTracers() {
    m_tracers.clear();
    for (int j = 0; j < 11; ++j) {
        for (int i = 0; i < 14; ++i) {
            Tracer p;
            p.position = Vec2f(0.08f + 0.055f * i, 0.12f + 0.07f * j);
            p.velocity = Vec2f(0.0f, 0.0f);
            m_tracers.push_back(p);
        }
    }
}

// Small Project 1-style cloth patch. Two upper corners are pinned; springs are
// structural plus diagonal shear springs, then the fluid velocity applies drag.
void FluidScene::buildCloth() {
    m_cloth.clear();
    m_clothSprings.clear();
    const int rows = 6;
    const int cols = 8;
    const float spacing = 0.025f;
    const Vec2f origin(0.18f, 0.85f);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            ClothNode node;
            node.position = origin + Vec2f(c * spacing, -r * spacing);
            node.velocity = Vec2f(0.0f, 0.0f);
            node.pinned = (r == 0 && (c == 0 || c == cols - 1));
            m_cloth.push_back(node);
        }
    }
    auto addSpring = [&](int a, int b) {
        ClothSpring s;
        s.a = a; s.b = b;
        s.rest = length(m_cloth[a].position - m_cloth[b].position);
        m_clothSprings.push_back(s);
    };
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int id = r * cols + c;
            if (c + 1 < cols) addSpring(id, id + 1);
            if (r + 1 < rows) addSpring(id, id + cols);
            if (c + 1 < cols && r + 1 < rows) addSpring(id, id + cols + 1);
            if (c > 0 && r + 1 < rows) addSpring(id, id + cols - 1);
        }
    }
}

Vec2f FluidScene::screenToUnit(int x, int y, int winX, int winY) const {
    return Vec2f(std::max(0.0f, std::min(1.0f, x / static_cast<float>(winX))),
                 std::max(0.0f, std::min(1.0f, 1.0f - y / static_cast<float>(winY))));
}

// Rebuild the solid boundary data from the current scene state.
// 1) clear previous solids/walls but keep outer walls;
// 2) add fixed internal grid-edge obstacles;
// 3) rasterize rigid bodies into solid cells and store their surface velocities
// for moving boundary conditions.
void FluidScene::rebuildSolids() {
    m_solver.clearSolids();
    if (m_enableFixedObjects) {
        m_solver.addBoxWall(37, 18, 45, 34);
        // A second arbitrary boundary made only out of marked edges, not solid cells.
        for (int k = 25; k <= 55; ++k) {
            int i = 18 + (k - 25) / 4;
            m_solver.setVerticalWall(i, k, true);
        }
    }

    if (!m_enableMovingSolids) return;
    const int N = m_solver.gridSize();
    for (const RigidBody2D& body : m_bodies) {
        if (body.fixed && !m_enableFixedObjects) continue;
        const float minX = std::max(0.0f, body.position[0] - body.radius - 0.02f);
        const float maxX = std::min(1.0f, body.position[0] + body.radius + 0.02f);
        const float minY = std::max(0.0f, body.position[1] - body.radius - 0.02f);
        const float maxY = std::min(1.0f, body.position[1] + body.radius + 0.02f);
        int i0 = std::max(1, static_cast<int>(minX * N));
        int i1 = std::min(N, static_cast<int>(maxX * N) + 1);
        int j0 = std::max(1, static_cast<int>(minY * N));
        int j1 = std::min(N, static_cast<int>(maxY * N) + 1);
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                Vec2f p((i - 0.5f) / N, (j - 0.5f) / N);
                if (body.contains(p)) {
                    Vec2f sv = body.surfaceVelocity(p);
                    m_solver.setSolidCell(i, j, true, sv[0], sv[1]);
                }
            }
        }
    }
}

// Approximate two-way coupling. A full method would integrate pressure and
// shear stress around the body. For the assignment demo we sample the fluid
// velocity around the body, compare it to the body's surface velocity, and use
// the difference as a drag impulse. The same samples create a torque.
void FluidScene::applyFluidForcesToBodies(float dt) {
    if (!m_enableTwoWayCoupling || !m_enableMovingSolids) return;
    for (RigidBody2D& body : m_bodies) {
        if (body.fixed || body.selected) continue;
        Vec2f totalImpulse(0.0f, 0.0f);
        float totalTorque = 0.0f;
        const int samples = 16;
        for (int s = 0; s < samples; ++s) {
            // Sample points around the body's surface
            Vec2f local;
            if (body.shape == RigidBody2D::BOX) {
                int edge = s / 4;
                float t = (s % 4 + 0.5f) / 4.0f * 2.0f - 1.0f;
                if (edge == 0) local = Vec2f(body.halfSize[0], body.halfSize[1] * t);
                else if (edge == 1) local = Vec2f(-body.halfSize[0], body.halfSize[1] * t);
                else if (edge == 2) local = Vec2f(body.halfSize[0] * t, body.halfSize[1]);
                else local = Vec2f(body.halfSize[0] * t, -body.halfSize[1]);
                float c = std::cos(body.angle), si = std::sin(body.angle);
                local = Vec2f(c * local[0] - si * local[1], si * local[0] + c * local[1]);
            } else {
                const float a = 2.0f * static_cast<float>(M_PI) * s / samples;
                local = Vec2f(std::cos(a) * body.radius, std::sin(a) * body.radius);
            }
            Vec2f point = body.position + local;
            Vec2f fluidVel = m_solver.sampleVelocity(point[0], point[1]);
            Vec2f rel = fluidVel - body.surfaceVelocity(point);
            Vec2f impulse = rel * (0.012f * body.mass);
            totalImpulse += impulse;
            totalTorque += cross2D(point - body.position, impulse);
        }
        body.applyImpulse(totalImpulse, body.position, m_enableRigidRotation);
        if (m_enableRigidRotation && !body.fixed) body.angularVelocity += totalTorque * body.invInertia;
    }
}

// Simple rigid-body contact. We use each body's bounding radius, separate
// overlapping objects, and apply a normal impulse when they are moving together.
// This matches the assignment permission to use impulses without resting contact.
void FluidScene::collideBodies() {
    if (!m_enableRigidCollisions || !m_enableMovingSolids) return;
    for (size_t a = 0; a < m_bodies.size(); ++a) {
        for (size_t b = a + 1; b < m_bodies.size(); ++b) {
            RigidBody2D& A = m_bodies[a];
            RigidBody2D& B = m_bodies[b];
            Vec2f delta = B.position - A.position;
            float dist = length(delta);
            float allowed = A.radius + B.radius;
            if (dist >= allowed || (A.fixed && B.fixed)) continue;
            
            Vec2f n;
            float penetration = 0.0f;

            // Calculate contact points and normal
            Vec2f contactPtA, contactPtB;
            // Circle circle
            if (A.shape == RigidBody2D::CIRCLE && B.shape == RigidBody2D::CIRCLE) {
                n = normalizeSafe(delta);
                penetration = allowed - dist;
                contactPtA = A.position + n * A.radius;
                contactPtB = B.position - n * B.radius;
            // Box box
            } else if (A.shape == RigidBody2D::BOX && B.shape == RigidBody2D::BOX) {
                Vec2f axesA[2], axesB[2];
                Vec2f cornersA[4], cornersB[4];
                A.getAxesAndCorners(axesA, cornersA);
                B.getAxesAndCorners(axesB, cornersB);
                
                Vec2f axesToTest[4] = { axesA[0], axesA[1], axesB[0], axesB[1] };
                float minOverlap = 1e9f;
                Vec2f bestAxis(0.0f, 0.0f);
                bool disjoint = false;
                
                // Separating Axis Theorem
                for (int i = 0; i < 4; ++i) {
                    Vec2f axis = axesToTest[i];
                    float minA = 1e9f, maxA = -1e9f;
                    float minB = 1e9f, maxB = -1e9f;
                    for (int k = 0; k < 4; ++k) {
                        float projA = cornersA[k] * axis;
                        minA = std::min(minA, projA); maxA = std::max(maxA, projA);
                        float projB = cornersB[k] * axis;
                        minB = std::min(minB, projB); maxB = std::max(maxB, projB);
                    }
                    if (maxA < minB || maxB < minA) {
                        disjoint = true;
                        break;
                    }
                    float overlap = std::min(maxA, maxB) - std::max(minA, minB);
                    if (overlap < minOverlap) {
                        minOverlap = overlap;
                        bestAxis = axis;
                    }
                }
                // no collision if disjoint
                if (disjoint) continue;
                penetration = minOverlap;
                n = (delta * bestAxis < 0) ? -bestAxis : bestAxis;
                Vec2f dummyN;
                A.closestSurfacePoint(B.position, contactPtA, dummyN);
                B.closestSurfacePoint(A.position, contactPtB, dummyN);
            } else {
                // Box circle
                RigidBody2D* box = (A.shape == RigidBody2D::BOX) ? &A : &B;
                RigidBody2D* circle = (A.shape == RigidBody2D::CIRCLE) ? &A : &B;
                Vec2f closestPt, normal;
                box->closestSurfacePoint(circle->position, closestPt, normal);
                
                // Check if circle is outside box and not overlapping
                Vec2f cDelta = circle->position - closestPt;
                float cDist = length(cDelta);
                
                if (cDist >= circle->radius && (cDelta * normal) >= 0.0f) continue;
                
                // Calculate contact normal and penetration depth
                if (cDist < 1e-6f) {
                    n = normal;
                    penetration = circle->radius;
                } else {
                    n = cDelta / cDist;
                    if (cDelta * normal < 0.0f) { // circle center is inside box
                        n = -n;
                        penetration = circle->radius + cDist;
                    } else {
                        penetration = circle->radius - cDist;
                    }
                }
                // Determine contact points on the surfaces
                if (box == &B) n = -n;
                if (box == &A) {
                    contactPtA = closestPt;
                    contactPtB = circle->position - n * circle->radius;
                } else {
                    contactPtA = circle->position + n * circle->radius;
                    contactPtB = closestPt;
                }
            }

            float invSum = A.invMass + B.invMass;
            if (invSum <= 0.0f) continue;
            
            if (!A.fixed) A.position -= n * (penetration * A.invMass / invSum);
            if (!B.fixed) B.position += n * (penetration * B.invMass / invSum);

            Vec2f rv = B.velocity - A.velocity;
            float vn = rv * n;
            if (vn < 0.0f) {
                float e = 0.45f;
                if (-vn < 0.05f) e = 0.0f; // prevent micro-bounces
                
                Vec2f rA = contactPtA - A.position; 
                Vec2f rB = contactPtB - B.position;
                float angA = m_enableRigidRotation ? cross2D(rA, n) * cross2D(rA, n) * A.invInertia : 0.0f;
                float angB = m_enableRigidRotation ? cross2D(rB, n) * cross2D(rB, n) * B.invInertia : 0.0f;
                float j = -(1.0f + e) * vn / (invSum + angA + angB);
                
                Vec2f impulse = n * j;
                A.applyImpulse(-impulse, contactPtA, m_enableRigidRotation);
                B.applyImpulse( impulse, contactPtB, m_enableRigidRotation);
            }
        }
    }
}

// Let tracer particles follow the fluid by relaxing their velocity toward the
// sampled grid velocity. Wrapping keeps them inside the demo window.
void FluidScene::stepTracers(float dt) {
    if (!m_enableParticlesAndCloth) return;
    for (Tracer& p : m_tracers) {
        Vec2f vf = m_solver.sampleVelocity(p.position[0], p.position[1]);
        p.velocity += (vf - p.velocity) * std::min(1.0f, 6.0f * dt);
        p.position += p.velocity * dt;
        
        // if tracer inside a rigid body push it out
        for (const RigidBody2D& body : m_bodies) {
            if (body.contains(p.position)) {
                Vec2f closestPt, normal;
                body.closestSurfacePoint(p.position, closestPt, normal);
                p.position = closestPt + normal * 1e-4f;
                float vn = p.velocity * normal;
                if (vn < 0.0f) {
                    p.velocity -= normal * vn;
                }
            }
        }
        
        // keep tracers inside the unit square
        if (p.position[0] < 0.02f) { p.position[0] = 0.02f; p.velocity[0] *= -0.5f; }
        if (p.position[0] > 0.98f) { p.position[0] = 0.98f; p.velocity[0] *= -0.5f; }
        if (p.position[1] < 0.02f) { p.position[1] = 0.02f; p.velocity[1] *= -0.5f; }
        if (p.position[1] > 0.98f) { p.position[1] = 0.98f; p.velocity[1] *= -0.5f; }
    }
}

// Mass-spring cloth with fluid drag. Each node receives spring forces, a small
// gravity-like force, and drag proportional to (fluidVelocity - nodeVelocity).
void FluidScene::stepCloth(float dt) {
    if (!m_enableParticlesAndCloth) return;
    std::vector<Vec2f> forces(m_cloth.size(), Vec2f(0.0f, 0.0f));
    const float ks = 150.0f;
    const float kd = 2.5f;
    for (const ClothSpring& s : m_clothSprings) {
        Vec2f x = m_cloth[s.a].position - m_cloth[s.b].position;
        Vec2f v = m_cloth[s.a].velocity - m_cloth[s.b].velocity;
        float len = length(x);
        if (len < 1e-6f) continue;
        Vec2f dir = x / len;
        Vec2f f = -(ks * (len - s.rest) + kd * (v * dir)) * dir;
        forces[s.a] += f;
        forces[s.b] -= f;
    }
    for (size_t i = 0; i < m_cloth.size(); ++i) {
        if (m_cloth[i].pinned) continue;

        // drag force between the fluid and the cloth node
        Vec2f fluid = m_solver.sampleVelocity(m_cloth[i].position[0], m_cloth[i].position[1]);
        Vec2f drag = (fluid - m_cloth[i].velocity) * 8.0f;
        
        // limit drag
        float dragLen = length(drag);
        if (dragLen > 2.0f) drag = drag * (2.0f / dragLen);
        
        // apply forces to the cloth node
        forces[i] += drag;
        forces[i] += Vec2f(0.0f, -0.15f);
        
        // apply drag to solver
        m_solver.addVelocityAt(m_cloth[i].position[0], m_cloth[i].position[1], -drag[0], -drag[1], 1);
        
        // node pos and vel update
        m_cloth[i].velocity += forces[i] * dt;
        m_cloth[i].velocity *= 0.995f;
        m_cloth[i].position += m_cloth[i].velocity * dt;
        
        // collision with rigid bodies
        for (const RigidBody2D& body : m_bodies) {
            if (body.contains(m_cloth[i].position)) {
                Vec2f closestPt, normal;
                body.closestSurfacePoint(m_cloth[i].position, closestPt, normal);
                m_cloth[i].position = closestPt + normal * 1e-4f;
                
                float vn = m_cloth[i].velocity * normal;
                if (vn < 0.0f) {
                    m_cloth[i].velocity -= normal * vn;
                }
            }
        }
        
        m_cloth[i].position[0] = std::max(0.02f, std::min(0.98f, m_cloth[i].position[0]));
        m_cloth[i].position[1] = std::max(0.02f, std::min(0.98f, m_cloth[i].position[1]));
    }
}

// Full scene step: move bodies, handle collisions, rebuild fluid boundaries,
// apply optional fluid-to-body impulses, advance the fluid, then advect
// particles/cloth through the resulting velocity field.
void FluidScene::step(float dt) {
    const auto sceneT0 = std::chrono::high_resolution_clock::now();
    m_solver.enableVorticity(m_enableVorticity);
    m_solver.enableBuoyancy(m_enableBuoyancy);

    for (RigidBody2D& body : m_bodies) {
        if (!body.selected && !body.fixed) body.integrate(dt, m_enableRigidRotation, m_enableGravity);
    }

    collideBodies();

    for (RigidBody2D& body : m_bodies) {
        if (!body.selected && !body.fixed) {
            if (body.shape == RigidBody2D::CIRCLE) {
                body.position[0] = std::max(body.radius, std::min(1.0f - body.radius, body.position[0]));
                body.position[1] = std::max(body.radius, std::min(1.0f - body.radius, body.position[1]));
            } else {
                Vec2f axes[2];
                Vec2f corners[4];
                body.getAxesAndCorners(axes, corners);
                float minX = 1e9f, maxX = -1e9f;
                float minY = 1e9f, maxY = -1e9f;
                for (int k = 0; k < 4; ++k) {
                    minX = std::min(minX, corners[k][0]);
                    maxX = std::max(maxX, corners[k][0]);
                    minY = std::min(minY, corners[k][1]);
                    maxY = std::max(maxY, corners[k][1]);
                }
                if (minX < 0.0f) body.position[0] -= minX;
                if (maxX > 1.0f) body.position[0] -= (maxX - 1.0f);
                if (minY < 0.0f) body.position[1] -= minY;
                if (maxY > 1.0f) body.position[1] -= (maxY - 1.0f);
            }
        }
    }

    rebuildSolids();
    applyFluidForcesToBodies(dt);
    
    // Set the surface velocity of each moving body into the fluid solver
    for (RigidBody2D& body : m_bodies) {
        // skip fixed and mouse dragged bodies
        if (body.fixed || body.selected) continue;
        
        // Compute the bounding box of the body in grid coordinates
        const int N = m_solver.gridSize();
        const float minX = std::max(0.0f, body.position[0] - body.radius - 0.02f);
        const float maxX = std::min(1.0f, body.position[0] + body.radius + 0.02f);
        const float minY = std::max(0.0f, body.position[1] - body.radius - 0.02f);
        const float maxY = std::min(1.0f, body.position[1] + body.radius + 0.02f);
        
        // Convert to grid indices
        int i0 = std::max(1, static_cast<int>(minX * N));
        int i1 = std::min(N, static_cast<int>(maxX * N) + 1);
        int j0 = std::max(1, static_cast<int>(minY * N));
        int j1 = std::min(N, static_cast<int>(maxY * N) + 1);
        
        // Set the surface velocity for each grid cell that is inside the body
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                Vec2f p((i - 0.5f) / N, (j - 0.5f) / N);
                if (body.contains(p)) {
                    Vec2f sv = body.surfaceVelocity(p);
                    m_solver.setSolidVelocity(i, j, sv[0], sv[1]);
                }
            }
        }
    }

    if (m_rightDown) {
        m_solver.addDensityAt(m_lastMouse[0], m_lastMouse[1], m_sourceAmount, 2);
        // Smoke injected by the user is hot, so it rises under buoyancy.
        m_solver.addTemperatureAt(m_lastMouse[0], m_lastMouse[1], m_heatAmount, 2);
    }
    m_solver.step(dt);
    stepTracers(dt);
    stepCloth(dt);

    // Average the full-scene cost and report both numbers periodically. Read the
    // console while toggling features to fill the report's timings table.
    const auto sceneT1 = std::chrono::high_resolution_clock::now();
    const double sceneMs = std::chrono::duration<double, std::milli>(sceneT1 - sceneT0).count();
    ++m_timingFrames;
    const double alpha = 0.05;
    m_avgSceneMs = (m_timingFrames == 1) ? sceneMs : (1.0 - alpha) * m_avgSceneMs + alpha * sceneMs;
    if (m_timingFrames % 60 == 0) {
        std::printf("[timing] solver %.3f ms | scene %.3f ms/step  "
                    "[vort=%d solids=%d couple=%d buoy=%d p&c=%d]\n",
                    m_solver.avgStepMs(), m_avgSceneMs,
                    m_enableVorticity ? 1 : 0, m_enableMovingSolids ? 1 : 0,
                    m_enableTwoWayCoupling ? 1 : 0, m_enableBuoyancy ? 1 : 0,
                    m_enableParticlesAndCloth ? 1 : 0);
    }
}

// Feature toggles. The assignment asks the demo to turn features on/off, so
// every required/extension feature has a key listed in printHelp().
bool FluidScene::handleKey(unsigned char key) {
    switch (key) {
    case 'v':
    case 'V':
        m_viewMode = (m_viewMode + 1) % VIEW_COUNT;
        std::printf("Fluid view: %s\n", viewName(m_viewMode));
        return true;
    case 'z':
    case 'Z':
        m_enableVorticity = !m_enableVorticity;
        std::printf("Vorticity confinement %s\n", m_enableVorticity ? "enabled" : "disabled");
        return true;
    case 'f':
    case 'F':
        m_enableFixedObjects = !m_enableFixedObjects;
        std::printf("Fixed internal boundaries %s\n", m_enableFixedObjects ? "enabled" : "disabled");
        rebuildSolids();
        return true;
    case 'm':
    case 'M':
        m_enableMovingSolids = !m_enableMovingSolids;
        std::printf("Moving solid objects %s\n", m_enableMovingSolids ? "enabled" : "disabled");
        rebuildSolids();
        return true;
    case 'r':
    case 'R':
        m_enableRigidRotation = !m_enableRigidRotation;
        std::printf("Rigid-body rotation %s\n", m_enableRigidRotation ? "enabled" : "disabled");
        return true;
    case 'b':
    case 'B':
        m_enableRigidCollisions = !m_enableRigidCollisions;
        std::printf("Rigid-body collision impulses %s\n", m_enableRigidCollisions ? "enabled" : "disabled");
        return true;
    case 'o':
    case 'O':
        m_enableTwoWayCoupling = !m_enableTwoWayCoupling;
        std::printf("Two-way coupling %s\n", m_enableTwoWayCoupling ? "enabled" : "disabled");
        return true;
    case 'p':
    case 'P':
        m_enableParticlesAndCloth = !m_enableParticlesAndCloth;
        std::printf("Particles and cloth %s\n", m_enableParticlesAndCloth ? "enabled" : "disabled");
        return true;
    case 't':
    case 'T':
        m_enableBuoyancy = !m_enableBuoyancy;
        m_solver.enableBuoyancy(m_enableBuoyancy);
        std::printf("Buoyancy: %s\n", m_enableBuoyancy ? "ON" : "OFF");
        return true;
    case 'g':
    case 'G':
        m_enableGravity = !m_enableGravity;
        std::printf("Rigid-body gravity %s\n", m_enableGravity ? "enabled" : "disabled");
        return true;
    case 'w':
    case 'W':
        if (m_solver.waterParticles().empty()) {
            m_solver.initWater(0.33f);
            std::printf("Free-surface water simulation enabled.\n");
        } else {
            m_solver.clearWater();
            std::printf("Free-surface water simulation disabled (normal smoke mode).\n");
        }
        return true;
    case 'c':
    case 'C':
        clear();
        std::printf("Fluid fields cleared.\n");
        return true;
    case 'h':
    case 'H':
        printHelp();
        return true;
    default:
        return false;
    }
}

// Mouse controls. Left dragging a body demonstrates moving solids; left dragging
// empty fluid injects velocity. Right dragging injects smoke density.
void FluidScene::mouse(int button, int state, int x, int y, int winX, int winY) {
    Vec2f p = screenToUnit(x, y, winX, winY);
    m_lastMouse = p;
    if (button == GLUT_LEFT_BUTTON) {
        m_leftDown = state == GLUT_DOWN;
        if (m_leftDown) {
            m_selectedBody = -1;
            for (int i = static_cast<int>(m_bodies.size()) - 1; i >= 0; --i) {
                if (m_bodies[i].contains(p) && !m_bodies[i].fixed && m_enableMovingSolids) {
                    m_selectedBody = i;
                    m_bodies[i].selected = true;
                    m_bodies[i].previousPosition = m_bodies[i].position;
                    m_bodies[i].previousAngle = m_bodies[i].angle;
                    m_grabOffset = m_bodies[i].position - p;
                    break;
                }
            }
        } else {
            if (m_selectedBody >= 0) {
                RigidBody2D& b = m_bodies[m_selectedBody];
                b.snapToGrid(m_solver.gridSize());
                b.selected = false;
                b.velocity = Vec2f(0.0f, 0.0f);
                // Requirement hook: dragged objects come to rest at exact grid offsets.
                b.angularVelocity = 0.0f;
                m_selectedBody = -1;
                rebuildSolids();
            }
        }
    } else if (button == GLUT_RIGHT_BUTTON) {
        m_rightDown = state == GLUT_DOWN;
        if (m_rightDown) {
            m_solver.addDensityAt(p[0], p[1], m_sourceAmount, 2);
            m_solver.addTemperatureAt(p[0], p[1], m_heatAmount, 2);
        }
    }
}

// Mouse motion while dragging. A moved rigid body immediately updates its
// velocity and rasterized solid cells, so the next fluid step sees a moving
// boundary and the body pushes the surrounding fluid.
void FluidScene::motion(int x, int y, int winX, int winY, float dt) {
    Vec2f p = screenToUnit(x, y, winX, winY);
    Vec2f delta = p - m_lastMouse;
    if (m_leftDown && m_selectedBody >= 0) {
        RigidBody2D& b = m_bodies[m_selectedBody];
        Vec2f old = b.position;
        float oldAngle = b.angle;
        b.position = p + m_grabOffset;
        if (b.shape == RigidBody2D::CIRCLE) {
            b.position[0] = std::max(b.radius, std::min(1.0f - b.radius, b.position[0]));
            b.position[1] = std::max(b.radius, std::min(1.0f - b.radius, b.position[1]));
        } else {
            Vec2f axes[2];
            Vec2f corners[4];
            b.getAxesAndCorners(axes, corners);
            float minX = 1e9f, maxX = -1e9f;
            float minY = 1e9f, maxY = -1e9f;
            for (int k = 0; k < 4; ++k) {
                minX = std::min(minX, corners[k][0]);
                maxX = std::max(maxX, corners[k][0]);
                minY = std::min(minY, corners[k][1]);
                maxY = std::max(maxY, corners[k][1]);
            }
            if (minX < 0.0f) b.position[0] -= minX;
            if (maxX > 1.0f) b.position[0] -= (maxX - 1.0f);
            if (minY < 0.0f) b.position[1] -= minY;
            if (maxY > 1.0f) b.position[1] -= (maxY - 1.0f);
        }
        b.velocity = (b.position - old) / dt;
        if (m_enableRigidRotation) {
            b.angle += delta[0] * 8.0f;
            b.angularVelocity = (b.angle - oldAngle) / dt;
        }
        // The moving solid immediately injects its velocity into the cells it occupies.
        rebuildSolids();
    } else if (m_leftDown) {
        m_solver.addVelocityAt(p[0], p[1], delta[0] * m_forceScale / dt, delta[1] * m_forceScale / dt, 2);
    }
    if (m_rightDown) {
        m_solver.addDensityAt(p[0], p[1], m_sourceAmount, 2);
        m_solver.addTemperatureAt(p[0], p[1], m_heatAmount, 2);
    }
    m_lastMouse = p;
}

void FluidScene::drawDensity() const {
    const int N = m_solver.gridSize();
    const float h = 1.0f / N;
    glBegin(GL_QUADS);
    for (int i = 0; i <= N; ++i) {
        float x = (i - 0.5f) * h;
        for (int j = 0; j <= N; ++j) {
            float y = (j - 0.5f) * h;
            float d00 = std::min(1.0f, m_solver.density(std::max(1, std::min(N, i)), std::max(1, std::min(N, j))) * 0.015f);
            float d01 = std::min(1.0f, m_solver.density(std::max(1, std::min(N, i)), std::max(1, std::min(N, j + 1))) * 0.015f);
            float d10 = std::min(1.0f, m_solver.density(std::max(1, std::min(N, i + 1)), std::max(1, std::min(N, j))) * 0.015f);
            float d11 = std::min(1.0f, m_solver.density(std::max(1, std::min(N, i + 1)), std::max(1, std::min(N, j + 1))) * 0.015f);
            glColor3f(d00, d00, d00); glVertex2f(x, y);
            glColor3f(d10, d10, d10); glVertex2f(x + h, y);
            glColor3f(d11, d11, d11); glVertex2f(x + h, y + h);
            glColor3f(d01, d01, d01); glVertex2f(x, y + h);
        }
    }
    glEnd();
}

void FluidScene::drawVelocity() const {
    const int N = m_solver.gridSize();
    const float h = 1.0f / N;
    glColor3f(0.8f, 0.9f, 1.0f);
    glBegin(GL_LINES);
    for (int i = 2; i <= N; i += 3) {
        for (int j = 2; j <= N; j += 3) {
            float x = (i - 0.5f) * h;
            float y = (j - 0.5f) * h;
            float u = m_solver.velocityU(i, j) * 0.015f;
            float v = m_solver.velocityV(i, j) * 0.015f;
            glVertex2f(x, y);
            glVertex2f(x + u, y + v);
        }
    }
    glEnd();
}

// Render one of the scalar diagnostic fields as a per-cell heatmap. The field is
// gathered once, the frame's peak magnitude is used to auto-scale, and each cell
// is drawn as a flat-shaded quad so the structure of the field is easy to read.
void FluidScene::drawScalarField(int mode) const {
    const int N = m_solver.gridSize();
    const float h = 1.0f / N;
    const bool signedField = (mode == VIEW_CURL || mode == VIEW_PRESSURE);

    std::vector<float> val(N * N, 0.0f);
    float maxMag = 1e-6f;
    for (int j = 1; j <= N; ++j) {
        for (int i = 1; i <= N; ++i) {
            float v = 0.0f;
            switch (mode) {
            case VIEW_CURL:        v = m_solver.curl(i, j); break;
            case VIEW_PRESSURE:    v = m_solver.pressure(i, j); break;
            case VIEW_TEMPERATURE: v = m_solver.temperature(i, j); break;
            default:               v = m_solver.density(i, j); break;
            }
            val[(j - 1) * N + (i - 1)] = v;
            maxMag = std::max(maxMag, std::fabs(v));
        }
    }


    glBegin(GL_QUADS);
    for (int j = 1; j <= N; ++j) {
        for (int i = 1; i <= N; ++i) {
            const float t = val[(j - 1) * N + (i - 1)] / maxMag;
            setHeatColor(t, signedField);
            const float x = (i - 1) * h;
            const float y = (j - 1) * h;
            glVertex2f(x, y); glVertex2f(x + h, y); glVertex2f(x + h, y + h); glVertex2f(x, y + h);
        }
    }
    glEnd();
}

void FluidScene::drawSolidGrid() const {
    const int N = m_solver.gridSize();
    const float h = 1.0f / N;
    const std::vector<unsigned char>& solid = m_solver.solidField();
    glColor3f(0.15f, 0.2f, 0.35f);
    glBegin(GL_QUADS);
    for (int j = 1; j <= N; ++j) {
        for (int i = 1; i <= N; ++i) {
            if (!solid[m_solver.ix(i, j)]) continue;
            float x = (i - 1) * h;
            float y = (j - 1) * h;
            glVertex2f(x, y); glVertex2f(x + h, y); glVertex2f(x + h, y + h); glVertex2f(x, y + h);
        }
    }
    glEnd();
}

void FluidScene::drawTracersAndCloth() const {
    if (!m_enableParticlesAndCloth) return;
    glPointSize(3.0f);
    glColor3f(0.3f, 1.0f, 0.55f);
    glBegin(GL_POINTS);
    for (const Tracer& p : m_tracers) glVertex2f(p.position[0], p.position[1]);
    glEnd();

    glColor3f(0.7f, 0.85f, 1.0f);
    glBegin(GL_LINES);
    for (const ClothSpring& s : m_clothSprings) {
        glVertex2f(m_cloth[s.a].position[0], m_cloth[s.a].position[1]);
        glVertex2f(m_cloth[s.b].position[0], m_cloth[s.b].position[1]);
    }
    glEnd();
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for (const ClothNode& n : m_cloth) glVertex2f(n.position[0], n.position[1]);
    glEnd();
}

void FluidScene::drawWaterParticles() const {
    const std::vector<Vec2f>& wp = m_solver.waterParticles();
    if (wp.empty()) return;
    const int N = m_solver.gridSize();
    const float h = 1.0f / N;
    glPointSize(3.0f);
    glColor3f(0.2f, 0.5f, 1.0f);
    glBegin(GL_POINTS);
    for (const Vec2f& p : wp) {
        glVertex2f((p[0] - 1.0f) * h, (p[1] - 1.0f) * h);
    }
    glEnd();
}

void FluidScene::draw() const {
    switch (m_viewMode) {
    case VIEW_VELOCITY:    drawVelocity(); break;
    case VIEW_CURL:        drawScalarField(VIEW_CURL); break;
    case VIEW_PRESSURE:    drawScalarField(VIEW_PRESSURE); break;
    case VIEW_TEMPERATURE: drawScalarField(VIEW_TEMPERATURE); break;
    default:               drawDensity(); break;
    }
    drawSolidGrid();
    if (m_enableMovingSolids || m_enableFixedObjects) {
        for (const RigidBody2D& b : m_bodies) {
            if (b.fixed && !m_enableFixedObjects) continue;
            if (!b.fixed && !m_enableMovingSolids) continue;
            b.draw();
        }
    }
    drawWaterParticles();
    drawTracersAndCloth();
}

void FluidScene::printHelp() const {
    std::printf("\n=== Project 2 Fluid Controls ===\n");
    std::printf("space - pause/run simulation\n");
    std::printf("left drag empty fluid - inject velocity\n");
    std::printf("left drag solid - move/rotate a rigid object; object snaps to grid on release\n");
    std::printf("right drag - add density/smoke\n");
    std::printf("v - cycle view: density / velocity / vorticity / pressure / temperature\n");
    std::printf("z - toggle vorticity confinement\n");
    std::printf("f - toggle fixed internal boundaries\n");
    std::printf("m - toggle moving solid objects\n");
    std::printf("r - toggle rigid rotation\n");
    std::printf("b - toggle rigid-body collision impulses\n");
    std::printf("o - toggle two-way fluid/solid coupling\n");
    std::printf("p - toggle tracer particles and cloth\n");
    std::printf("t - toggle temperature buoyancy (hot smoke rises)\n");
    std::printf("g - toggle rigid-body gravity\n");
    std::printf("w - toggle free-surface water simulation (marker particles)\n");
    std::printf("c - clear fluid fields\n");
    std::printf("s - cycle back to Project 1 scenes\n");
    std::printf("================================\n\n");
}
