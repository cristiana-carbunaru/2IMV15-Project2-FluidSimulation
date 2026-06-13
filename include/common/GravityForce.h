#ifndef INC_2IMV15_PROJECT1_GRAVITYFORCE_H
#define INC_2IMV15_PROJECT1_GRAVITYFORCE_H

#endif //INC_2IMV15_PROJECT1_GRAVITYFORCE_H

#pragma once
#include "Force.h"
#include "Particle.h"
#include <vector>

class GravityForce : public Force {
public:
    GravityForce(const std::vector<Particle*>& particles, const Vec2f& gravity);
    void apply() override;
    void draw() override;

private:
    std::vector<Particle*> m_particles;
    Vec2f m_gravity;
};
