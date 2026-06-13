#include "Force.h"
#include "Particle.h"

class MouseSpringForce : public Force
{
public:
    MouseSpringForce(Particle* particle, float ks, float kd);
    void updateMousePosition(float x, float y) { m_mousePos = Vec2f(x, y); };
    void apply() override;
    void draw() override;

private:
    Particle* m_particle;
    Vec2f m_mousePos;
    double m_ks, m_kd;
};