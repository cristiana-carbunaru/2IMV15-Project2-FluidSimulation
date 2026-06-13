#include "WindForce.h"
#include "GLCompat.h"
#include <cmath>

WindForce::WindForce(
    const std::vector<Particle*>& particles, const Vec2f& windDirection, float windStrength, bool enabled)
    : m_particles(particles), m_wind(windDirection), m_strength(windStrength) {
    m_enabled = enabled;
}

void WindForce::apply() {
    if (!m_enabled) return;

    // Gust oscillation: slowly varying envelope makes cloth billow naturally
    m_phase += 0.015f;
    float effective = m_strength * (1.0f + 0.4f * sinf(m_phase));

    for (Particle* p : m_particles) {
        p->m_Force += effective * m_wind;
    }
}

void WindForce::draw() {
    // wind force as lines
    if (m_enabled) {
        for (Particle* p : m_particles) {
            Vec2f start = p->m_Position;
            Vec2f end = start + m_strength * m_wind; // scale the wind vector for visualization

            glBegin(GL_LINES);
            glColor3f(0.5, 1.0, 0.5); // light green
            glVertex2f(start[0], start[1]);
            glVertex2f(end[0], end[1]);
            glEnd();
        }
    }
}