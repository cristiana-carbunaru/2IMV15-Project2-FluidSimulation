#define POINT_CONSTRAINT_H
#include "Constraint.h"

class PointConstraintX : public Constraint
{
public:
    PointConstraintX(Particle* p, double target_x);

    double C() const override;

    double C_dot() const override;

    std::vector<Particle*> getParticles() const override;

    std::vector<Vec2f> J_rows() const override;

    std::vector<Vec2f> J_dot_rows() const override;
    
    void draw() override;

private:
    Particle* m_p;
    double const m_target;
};

class PointConstraintY : public Constraint
{
public:
    PointConstraintY(Particle* p, double target_y);

    double C() const override;

    double C_dot() const override;
    
    std::vector<Particle*> getParticles() const override;
    
    std::vector<Vec2f> J_rows() const override;
    
    std::vector<Vec2f> J_dot_rows() const override;
    
    void draw() override;

private:
    Particle* m_p;
    double const m_target;
};