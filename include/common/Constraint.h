#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "Particle.h"
#include <vector>

class Constraint
{
public:
    virtual ~Constraint() {}

    virtual double C() const = 0;

    virtual double C_dot() const = 0;

    virtual std::vector<Particle*> getParticles() const = 0;

    virtual std::vector<Vec2f> J_rows() const = 0;

    virtual std::vector<Vec2f> J_dot_rows() const = 0;

    virtual void draw() = 0;

    void setEnabled(bool enabled) { m_enabled = enabled; }
    bool isEnabled() const { return m_enabled; }

protected:
    bool m_enabled = true;
};

#endif