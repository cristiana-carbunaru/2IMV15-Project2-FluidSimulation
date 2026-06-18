#pragma once

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
    Vec2f surfaceVelocity(const Vec2f& worldPoint) const;
    void integrate(float dt, bool allowRotation, bool applyGravity);
    void applyImpulse(const Vec2f& impulse, const Vec2f& contactPoint, bool allowRotation);
    void snapToGrid(int N);
    void closestSurfacePoint(const Vec2f& p, Vec2f& outClosest, Vec2f& outNormal) const;
    void getAxesAndCorners(Vec2f axes[2], Vec2f corners[4]) const;
    void draw() const;
};
