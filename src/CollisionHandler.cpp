#include "CollisionHandler.h"
#include "GLCompat.h"
#include <cmath>

void CollisionHandler::handleWallCollisions(std::vector<Particle*>& particles, const std::vector<Wall>& walls, float restitution, float friction) {
    for (const Wall& wall : walls) {
        Vec2f edge = wall.b - wall.a;
        float edgeLenSq = edge * edge;
        if (edgeLenSq < 1e-8f) continue;

        for (Particle* p : particles) {
            Vec2f X = p->m_Position;
            float t = ((X - wall.a) * edge) / edgeLenSq;
            if (t < 0.0f || t > 1.0f) continue;
            Vec2f closest = wall.a + edge * t;
            Vec2f diff = X - closest;
            float dist = diff * wall.normal;

            if (dist < 0.0f) {
                p->m_Position -= dist * wall.normal;

                Vec2f v = p->m_Velocity;
                float v_dot_n = v[0] * wall.normal[0] + v[1] * wall.normal[1];
                
                // Only bounce if moving towards the wall
                if (v_dot_n < 0.0f) {
                    Vec2f v_n(v_dot_n * wall.normal[0], v_dot_n * wall.normal[1]);
                    Vec2f v_t(v[0] - v_n[0], v[1] - v_n[1]);
                    v_t[0] *= (1.0f - friction);
                    v_t[1] *= (1.0f - friction);
                    
                    p->m_Velocity = v_t - restitution * v_n;
                }
            }
        }
    }
}

void CollisionHandler::handleParticleCollisions(std::vector<Particle*>& particles, float particle_diameter, float restitution) {
    const float particle_diameter_sq = particle_diameter * particle_diameter;

    for (size_t i = 0; i < particles.size(); ++i) {
        for (size_t j = i + 1; j < particles.size(); ++j) {
            Particle* p1 = particles[i];
            Particle* p2 = particles[j];
            if (p1->m_Pinned || p2->m_Pinned) continue;  // Never disturb pinned anchors

            Vec2f delta = p2->m_Position - p1->m_Position;
            float dist_sq = delta[0] * delta[0] + delta[1] * delta[1];

            if (dist_sq < particle_diameter_sq) {
                float dist = sqrt(dist_sq);
                // Avoid division by zero
                if (dist == 0.0f) {
                    dist = 0.001f;
                    delta = Vec2f(0.001f, 0.0f); // Arbitrary direction
                }
                Vec2f collision_normal = delta / dist;

                // Pos correction
                float penetration_depth = particle_diameter - dist;
                Vec2f correction = collision_normal * (penetration_depth / 2.0f);
                p1->m_Position -= correction;
                p2->m_Position += correction;

                // Vel correction
                Vec2f relative_velocity = p2->m_Velocity - p1->m_Velocity;
                float vel_along_normal = relative_velocity[0] * collision_normal[0] + relative_velocity[1] * collision_normal[1];

                if (vel_along_normal > 0) continue; // Already separating

                float impulse_scalar = -(1 + restitution) * vel_along_normal / 2.0f; // Divide by 2 for equal mass
                Vec2f impulse = impulse_scalar * collision_normal;

                p1->m_Velocity -= impulse;
                p2->m_Velocity += impulse;
            }
        }
    }
}

void CollisionHandler::drawWalls(const std::vector<Wall>& walls) {
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    for (const Wall& wall : walls) {
        glVertex2f(wall.a[0], wall.a[1]);
        glVertex2f(wall.b[0], wall.b[1]);
    }
    glEnd();
}
