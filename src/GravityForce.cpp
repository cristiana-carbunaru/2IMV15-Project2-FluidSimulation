#include "GravityForce.h"

GravityForce::GravityForce(const std::vector<Particle*>& particles, const Vec2f& gravity)
    : m_particles(particles), m_gravity(gravity) {}

void GravityForce::apply() {
    for (Particle* p : m_particles) {
        p->m_Force += p->m_Mass * m_gravity;
    }
}

void GravityForce::draw() {
}
