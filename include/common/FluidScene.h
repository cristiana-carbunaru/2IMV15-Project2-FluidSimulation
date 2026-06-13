#pragma once

// FluidScene:
// This class owns the fluid solver, rigid bodies, tracer particles, and a small cloth patch.
// Keeping this separate from FluidSolver2D lets the numerical solver stay readable while
// the scene handles controls, drawing, and feature toggles required by the assignment.

#include "FluidSolver2D.h"
#include "RigidBody2D.h"
#include <vector>

class FluidScene {
public:
    FluidScene();
    void init();
    void clear();
    void step(float dt);
    void draw() const;
    bool handleKey(unsigned char key);
    void mouse(int button, int state, int x, int y, int winX, int winY);
    void motion(int x, int y, int winX, int winY);
    void printHelp() const;

private:
    struct Tracer {
        Vec2f position;
        Vec2f velocity;
    };
    struct ClothNode {
        Vec2f position;
        Vec2f velocity;
        bool pinned;
    };
    struct ClothSpring {
        int a, b;
        float rest;
    };

    FluidSolver2D m_solver;
    std::vector<RigidBody2D> m_bodies;
    std::vector<Tracer> m_tracers;
    std::vector<ClothNode> m_cloth;
    std::vector<ClothSpring> m_clothSprings;

    bool m_drawVelocity;
    bool m_enableVorticity;
    bool m_enableFixedObjects;
    bool m_enableMovingSolids;
    bool m_enableRigidRotation;
    bool m_enableRigidCollisions;
    bool m_enableTwoWayCoupling;
    bool m_enableParticlesAndCloth;
    int m_selectedBody;
    bool m_leftDown;
    bool m_rightDown;
    Vec2f m_lastMouse;
    Vec2f m_grabOffset;
    float m_sourceAmount;
    float m_forceScale;

    Vec2f screenToUnit(int x, int y, int winX, int winY) const;
    // Re-render fixed walls and moving rigid bodies into FluidSolver2D.
    // Called after body motion so the fluid sees the current solid geometry.
    void rebuildSolids();
    // Approximate two-way coupling: sample fluid velocity around each body and
    // apply a drag-like impulse/torque back to the rigid body.
    void applyFluidForcesToBodies(float dt);
    // Simple impulse/separation collision handling for boxes/disks. Resting
    // contact is intentionally not implemented, matching the assignment option.
    void collideBodies();
    void seedTracers();
    void buildCloth();
    void stepTracers(float dt);
    // Similar to Project 1, mass-spring cloth driven by sampled fluid velocity.
    void stepCloth(float dt);
    void drawDensity() const;
    void drawVelocity() const;
    void drawSolidGrid() const;
    void drawTracersAndCloth() const;
};
