// RigidBody2D.cpp:
// Rigid body helper for the coupled fluid scene. The implementation uses the
// basic formulas from rigid-body lectures: mass/inertia, explicit integration,
// surface velocity, and impulses. It is intentionally lightweight and demo-
// oriented.

#include "RigidBody2D.h"
#include "GLCompat.h"
#include <algorithm>

static Vec2f rotateVec(const Vec2f& v, float a) {
    const float c = std::cos(a);
    const float s = std::sin(a);
    return Vec2f(c * v[0] - s * v[1], s * v[0] + c * v[1]);
}

static float cross2(const Vec2f& a, const Vec2f& b) {
    return a[0] * b[1] - a[1] * b[0];
}

// Box inertia about its centre: I = m(w^2 + h^2)/12.
RigidBody2D RigidBody2D::makeBox(const Vec2f& center, const Vec2f& halfExtents, float m, bool isFixed) {
    RigidBody2D b;
    b.shape = BOX;
    b.position = b.previousPosition = center;
    b.velocity = Vec2f(0.0f, 0.0f);
    b.halfSize = halfExtents;
    b.radius = std::sqrt(halfExtents[0] * halfExtents[0] + halfExtents[1] * halfExtents[1]);
    b.angle = b.previousAngle = 0.0f;
    b.angularVelocity = 0.0f;
    b.mass = std::max(0.001f, m);
    b.fixed = isFixed;
    b.invMass = isFixed ? 0.0f : 1.0f / b.mass;
    const float w = 2.0f * halfExtents[0];
    const float h = 2.0f * halfExtents[1];
    b.inertia = b.mass * (w * w + h * h) / 12.0f;
    b.invInertia = isFixed ? 0.0f : 1.0f / b.inertia;
    b.selected = false;
    return b;
}

// Disk inertia about its centre: I = 1/2 m r^2.
RigidBody2D RigidBody2D::makeCircle(const Vec2f& center, float r, float m, bool isFixed) {
    RigidBody2D b;
    b.shape = CIRCLE;
    b.position = b.previousPosition = center;
    b.velocity = Vec2f(0.0f, 0.0f);
    b.halfSize = Vec2f(r, r);
    b.radius = r;
    b.angle = b.previousAngle = 0.0f;
    b.angularVelocity = 0.0f;
    b.mass = std::max(0.001f, m);
    b.fixed = isFixed;
    b.invMass = isFixed ? 0.0f : 1.0f / b.mass;
    b.inertia = 0.5f * b.mass * r * r;
    b.invInertia = isFixed ? 0.0f : 1.0f / b.inertia;
    b.selected = false;
    return b;
}

// Point-in-body test used to rasterize the object into occupied fluid cells and
// to detect mouse picking. Boxes are tested in local coordinates.
bool RigidBody2D::contains(const Vec2f& p) const {
    const Vec2f q = rotateVec(p - position, -angle);
    if (shape == CIRCLE) return q * q <= radius * radius;
    return std::fabs(q[0]) <= halfSize[0] && std::fabs(q[1]) <= halfSize[1];
}

// Moving solid boundary velocity imposed on the fluid. Rotation matters here:
// without omega x r, rotating bodies would not drag fluid tangentially.
Vec2f RigidBody2D::surfaceVelocity(const Vec2f& worldPoint) const {
    const Vec2f r = worldPoint - position;
    return velocity + Vec2f(-angularVelocity * r[1], angularVelocity * r[0]);
}

// Explicit Euler integration for demo motion. Boundaries bounce weakly so the
// bodies remain inside the fluid domain.
void RigidBody2D::integrate(float dt, bool allowRotation, bool applyGravity) {
    previousPosition = position;
    previousAngle = angle;
    if (fixed) return;
    // gravity
    if (applyGravity) {
        velocity[1] -= 0.5f * dt;
    }
    position += velocity * dt;
    if (allowRotation) angle += angularVelocity * dt;
    else angularVelocity = 0.0f;

    // Keep demo objects in the fluid box.
    if (shape == CIRCLE) {
        if (position[0] < radius) { position[0] = radius; velocity[0] *= -0.3f; }
        if (position[0] > 1.0f - radius) { position[0] = 1.0f - radius; velocity[0] *= -0.3f; }
        if (position[1] < radius) { position[1] = radius; velocity[1] *= -0.3f; }
        if (position[1] > 1.0f - radius) { position[1] = 1.0f - radius; velocity[1] *= -0.3f; }
    } else {
        Vec2f axes[2];
        Vec2f corners[4];
        getAxesAndCorners(axes, corners);
        float minX = 1e9f, maxX = -1e9f;
        float minY = 1e9f, maxY = -1e9f;
        for (int k = 0; k < 4; ++k) {
            minX = std::min(minX, corners[k][0]);
            maxX = std::max(maxX, corners[k][0]);
            minY = std::min(minY, corners[k][1]);
            maxY = std::max(maxY, corners[k][1]);
        }
        if (minX < 0.0f) { position[0] -= minX; if (velocity[0] < 0.0f) velocity[0] *= -0.3f; }
        if (maxX > 1.0f) { position[0] -= (maxX - 1.0f); if (velocity[0] > 0.0f) velocity[0] *= -0.3f; }
        if (minY < 0.0f) { position[1] -= minY; if (velocity[1] < 0.0f) velocity[1] *= -0.3f; }
        if (maxY > 1.0f) { position[1] -= (maxY - 1.0f); if (velocity[1] > 0.0f) velocity[1] *= -0.3f; }
    }
}

// Linear impulse changes v by J/m; angular impulse changes omega by
// I^{-1}(r x J). This is reused for collision response and fluid drag torque.
void RigidBody2D::applyImpulse(const Vec2f& impulse, const Vec2f& contactPoint, bool allowRotation) {
    if (fixed) return;
    velocity += impulse * invMass;
    if (allowRotation) {
        const Vec2f r = contactPoint - position;
        angularVelocity += cross2(r, impulse) * invInertia;
    }
}

// Snap the centre to the nearest cell spacing 1/N. This is called when a
// user releases a dragged object, matching the assignment requirement.
void RigidBody2D::snapToGrid(int N) {
    const float h = 1.0f / N;
    position[0] = (std::floor(position[0] / h + 0.5f)) * h;
    position[1] = (std::floor(position[1] / h + 0.5f)) * h;
}

// Find closest point on the surface and outward normal
void RigidBody2D::closestSurfacePoint(const Vec2f& p, Vec2f& outClosest, Vec2f& outNormal) const {
    // closest point is along the radius, normal is radial
    if (shape == CIRCLE) {
        Vec2f delta = p - position;
        float dist = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1]);
        if (dist < 1e-6f) {
            outNormal = Vec2f(1.0f, 0.0f);
            outClosest = position + outNormal * radius;
        } else {
            outNormal = delta / dist;
            outClosest = position + outNormal * radius;
        }
        return;
    }
    
    // BOX
    Vec2f q = rotateVec(p - position, -angle);
    Vec2f q_clamped(std::max(-halfSize[0], std::min(halfSize[0], q[0])),
                    std::max(-halfSize[1], std::min(halfSize[1], q[1])));
    
    // If p is inside the box, push it to the nearest edge
    bool inside = false;
    if (std::fabs(q[0]) < halfSize[0] && std::fabs(q[1]) < halfSize[1]) {
        inside = true;
        float dx = halfSize[0] - std::fabs(q[0]);
        float dy = halfSize[1] - std::fabs(q[1]);
        if (dx < dy) {
            q_clamped[0] = (q[0] > 0) ? halfSize[0] : -halfSize[0];
        } else {
            q_clamped[1] = (q[1] > 0) ? halfSize[1] : -halfSize[1];
        }
    }
    
    // Calculate local normal and closest point in world coordinates
    Vec2f localNormal = inside ? (q_clamped - q) : (q - q_clamped);
    float dist = std::sqrt(localNormal[0] * localNormal[0] + localNormal[1] * localNormal[1]);
    if (dist < 1e-6f) {
        if (std::fabs(q_clamped[0]) == halfSize[0]) {
            localNormal = Vec2f((q_clamped[0] > 0) ? 1.0f : -1.0f, 0.0f);
        } else {
            localNormal = Vec2f(0.0f, (q_clamped[1] > 0) ? 1.0f : -1.0f);
        }
    } else {
        localNormal = localNormal / dist;
    }
    
    // back to world space
    outNormal = rotateVec(localNormal, angle);
    outClosest = position + rotateVec(q_clamped, angle);
}

// Calculate orientation axes and corners
void RigidBody2D::getAxesAndCorners(Vec2f axes[2], Vec2f corners[4]) const {
    // apply rotation to local axes to get world axes
    axes[0] = rotateVec(Vec2f(1.0f, 0.0f), angle);
    axes[1] = rotateVec(Vec2f(0.0f, 1.0f), angle);

    // Local corners before rotation
    Vec2f localCorners[4] = {
        Vec2f(-halfSize[0], -halfSize[1]), Vec2f( halfSize[0], -halfSize[1]),
        Vec2f( halfSize[0],  halfSize[1]), Vec2f(-halfSize[0],  halfSize[1])
    };
    // Rotate and translate to world space
    for (int k = 0; k < 4; ++k) {
        corners[k] = position + rotateVec(localCorners[k], angle);
    }
}

// Immediate-mode OpenGL drawing. The line/spoke shows orientation so rotation
// is visible in the demo video.
void RigidBody2D::draw() const {
    if (selected) glColor3f(1.0f, 0.8f, 0.2f);
    else if (fixed) glColor3f(0.25f, 0.45f, 0.95f);
    else glColor3f(0.95f, 0.35f, 0.25f);

    if (shape == CIRCLE) {
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(position[0], position[1]);
        for (int k = 0; k <= 40; ++k) {
            const float a = 2.0f * static_cast<float>(M_PI) * k / 40.0f;
            glVertex2f(position[0] + std::cos(a) * radius, position[1] + std::sin(a) * radius);
        }
        glEnd();
        glColor3f(0.05f, 0.05f, 0.05f);
        glBegin(GL_LINES);
        glVertex2f(position[0], position[1]);
        Vec2f spoke = rotateVec(Vec2f(radius, 0.0f), angle);
        glVertex2f(position[0] + spoke[0], position[1] + spoke[1]);
        glEnd();
        return;
    }

    Vec2f corners[4] = {
        Vec2f(-halfSize[0], -halfSize[1]), Vec2f( halfSize[0], -halfSize[1]),
        Vec2f( halfSize[0],  halfSize[1]), Vec2f(-halfSize[0],  halfSize[1])
    };
    glBegin(GL_QUADS);
    for (int k = 0; k < 4; ++k) {
        Vec2f c = position + rotateVec(corners[k], angle);
        glVertex2f(c[0], c[1]);
    }
    glEnd();
    glColor3f(0.05f, 0.05f, 0.05f);
    glBegin(GL_LINES);
    Vec2f a = position + rotateVec(Vec2f(-halfSize[0], 0.0f), angle);
    Vec2f b = position + rotateVec(Vec2f( halfSize[0], 0.0f), angle);
    glVertex2f(a[0], a[1]); glVertex2f(b[0], b[1]);
    glEnd();
}
