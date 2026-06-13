#pragma once
#include "Force.h"
#include "Particle.h"
#include <vector>

class TouchSphereForce : public Force {
public:
    TouchSphereForce(const std::vector<Particle*>& particles, float radius, float maxForce);
    void updateMousePosition(float x, float y);
    void apply() override;
    void draw() override;

private:
    std::vector<Particle*> m_particles;
    Vec2f m_mousePos;
    float m_radius;
    float m_maxForce;
};