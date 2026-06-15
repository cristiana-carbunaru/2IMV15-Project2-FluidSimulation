#pragma once

// RigidBody2D:
// Minimal rigid-body data used by the fluid demo. Bodies can be boxes or disks,
// can translate and rotate, and can receive collision/fluid impulses. The class
// deliberately stays simple because the assignment only requires simple rigid
// objects and impulse-based separation, not a full rigid-body engine.


#include <cmath>
#include <gfx/vec2.h>

class RigidBody2D {
public:
    enum Shape { BOX, CIRCLE };

    Shape shape;
    Vec2f position;
    Vec2f previousPosition;
    Vec2f velocity;
    Vec2f halfSize;
    float radius;
    float angle;
    float previousAngle;
    float angularVelocity;
    float mass;
    float invMass;
    float inertia;
    float invInertia;
    bool fixed;
    bool selected;

    static RigidBody2D makeBox(const Vec2f& center, const Vec2f& halfExtents, float mass, bool fixed = false);
    static RigidBody2D makeCircle(const Vec2f& center, float radius, float mass, bool fixed = false);

    bool contains(const Vec2f& p) const;
    // Velocity of a point on/in the rigid body: linear velocity plus the 2D
    // angular contribution omega x r = (-omega*r_y, omega*r_x).
    Vec2f surfaceVelocity(const Vec2f& worldPoint) const;
    void integrate(float dt, bool allowRotation, bool applyGravity);
    // Apply an impulse at a contact/sample point. If rotation is enabled, the
    // impulse also changes angular velocity through torque r x impulse.
    void applyImpulse(const Vec2f& impulse, const Vec2f& contactPoint, bool allowRotation);
    // Assignment requirement for dragged moving solids: after release they come
    // to rest at an exact grid offset.
    void snapToGrid(int N);
    void closestSurfacePoint(const Vec2f& p, Vec2f& outClosest, Vec2f& outNormal) const;
    void getAxesAndCorners(Vec2f axes[2], Vec2f corners[4]) const;
    void draw() const;
};
