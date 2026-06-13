#pragma once

#include "Constraint.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);

  double C() const override;
  double C_dot() const override;
  std::vector<Particle*> getParticles() const override;
  std::vector<Vec2f> J_rows() const override;
  std::vector<Vec2f> J_dot_rows() const override;
  void draw() override;

 private:
  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};