#pragma once
#include "Force.h"
#include "Particle.h"
#include <vector>

class WindForce : public Force {
public:
    WindForce(const std::vector<Particle*>& particles, const Vec2f& windDirection, float windStrength, bool enabled);
    void apply() override;
    void draw() override;

private:
    std::vector<Particle*> m_particles;
    Vec2f m_wind;
    float m_strength;
    float m_phase = 0.0f;  // for oscillating gust effect
};