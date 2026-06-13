#include "MouseSpringForce.h"
#include "GLCompat.h"
#include <cmath>

MouseSpringForce::MouseSpringForce(Particle* particle, float ks, float kd) : 
    m_particle(particle), m_mousePos(Vec2f(0.0f, 0.0f)), m_ks(ks), m_kd(kd) 
{}

void MouseSpringForce::apply() {
    Vec2f l = m_particle->m_Position - m_mousePos; // vector from mouse to particle
    float dist = sqrt(l[0] * l[0] + l[1] * l[1]);
    if (dist < 1e-6) return; // avoid division by zero
    
    // Hooke's law
    Vec2f l_normalized = l / dist;
    float force_magnitude = -(m_ks * (dist - 0.0f) + m_kd * (m_particle->m_Velocity * l_normalized));
    m_particle->m_Force += force_magnitude * l_normalized;
}

void MouseSpringForce::draw() {
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 1.0);
    glVertex2f(m_particle->m_Position[0], m_particle->m_Position[1]);
    glVertex2f(m_mousePos[0], m_mousePos[1]);
    glEnd();
}