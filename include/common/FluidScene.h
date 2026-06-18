#pragma once

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
    void motion(int x, int y, int winX, int winY, float dt);
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

    // Visualization view cycled with the 'v' key. Values match the ViewMode
    int m_viewMode;
    bool m_enableVorticity;
    bool m_enableFixedObjects;
    bool m_enableMovingSolids;
    bool m_enableRigidRotation;
    bool m_enableRigidCollisions;
    bool m_enableTwoWayCoupling;
    bool m_enableParticlesAndCloth;
    bool m_enableBuoyancy;
    bool m_enableGravity;
    int m_selectedBody;
    bool m_leftDown;
    bool m_rightDown;
    Vec2f m_lastMouse;
    Vec2f m_grabOffset;
    float m_sourceAmount;
    float m_forceScale;
    float m_heatAmount;

    double m_avgSceneMs = 0.0;
    long m_timingFrames = 0;

    Vec2f screenToUnit(int x, int y, int winX, int winY) const;
    void rebuildSolids();
    void applyFluidForcesToBodies(float dt);
    void collideBodies();
    void seedTracers();
    void buildCloth();
    void stepTracers(float dt);
    // Similar to Project 1, mass-spring cloth driven by sampled fluid velocity.
    void stepCloth(float dt);
    void drawDensity() const;
    void drawVelocity() const;
    // Generic heatmap render
    void drawScalarField(int mode) const;
    void drawSolidGrid() const;
    void drawTracersAndCloth() const;
    void drawWaterParticles() const;
};
