#include "TouchSphereForce.h"
#include "GLCompat.h"
#include <cmath>

TouchSphereForce::TouchSphereForce(const std::vector<Particle*>& particles, float radius, float maxForce)
    : m_particles(particles), m_mousePos(Vec2f(0, 0)), m_radius(radius), m_maxForce(maxForce) {}

void TouchSphereForce::updateMousePosition(float x, float y) {
    m_mousePos = Vec2f(x, y);
}

void TouchSphereForce::apply() {
    for (Particle* p : m_particles) {
        if (p->m_Pinned) continue;

        Vec2f dir = p->m_Position - m_mousePos;
        float dist = sqrt(dir[0] * dir[0] + dir[1] * dir[1]);

        if (dist > 0.0f && dist < m_radius) {
            dir = dir / dist; // normalize
            
            // make force strongest at center and weakest at radius
            float falloff = 1.0f - (dist / m_radius);
            falloff = falloff * falloff; 
            
            p->m_Force += dir * (m_maxForce * falloff);
        }
    }
}

void TouchSphereForce::draw() {
    // interaction radius
    glBegin(GL_LINE_LOOP);
    glColor3f(0.2f, 0.6f, 1.0f);
    for (int i = 0; i < 36; ++i) {
        float theta = i * 2.0f * (float)M_PI / 36.0f;
        glVertex2f(m_mousePos[0] + m_radius * cos(theta), 
                   m_mousePos[1] + m_radius * sin(theta));
    }
    glEnd();
}