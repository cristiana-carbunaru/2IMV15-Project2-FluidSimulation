#include "PointConstraint.h"
#include "GLCompat.h"

PointConstraintX::PointConstraintX(Particle* p, double target_x) : m_p(p), m_target(target_x) {}

double PointConstraintX::C() const {
    return m_p->m_Position[0] - m_target;
}

double PointConstraintX::C_dot() const {
    return m_p->m_Velocity[0];
}

std::vector<Particle*> PointConstraintX::getParticles() const {
    return {m_p};
}

std::vector<Vec2f> PointConstraintX::J_rows() const {
    return {Vec2f(1.0f, 0.0f)};
}

std::vector<Vec2f> PointConstraintX::J_dot_rows() const {
    return {Vec2f(0.0f, 0.0f)};
}

void PointConstraintX::draw() {
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f(m_target, m_p->m_Position[1] - 0.025f);
    glVertex2f(m_target, m_p->m_Position[1] + 0.025f);
    glEnd();
}

PointConstraintY::PointConstraintY(Particle* p, double target_y) : m_p(p), m_target(target_y) {}

double PointConstraintY::C() const {
    return m_p->m_Position[1] - m_target;
}

double PointConstraintY::C_dot() const {
    return m_p->m_Velocity[1];
}

std::vector<Particle*> PointConstraintY::getParticles() const {
    return {m_p};
}

std::vector<Vec2f> PointConstraintY::J_rows() const {
    return {Vec2f(0.0f, 1.0f)};
}

std::vector<Vec2f> PointConstraintY::J_dot_rows() const {
    return {Vec2f(0.0f, 0.0f)};
}

void PointConstraintY::draw() {
    glBegin(GL_LINES);
    glColor3f(0.0, 1.0, 0.0);
    glVertex2f(m_p->m_Position[0] - 0.025f, m_target);
    glVertex2f(m_p->m_Position[0] + 0.025f, m_target);
    glEnd();
}