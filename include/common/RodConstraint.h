#pragma once

#include "Constraint.h"

class RodConstraint : public Constraint {
 public:
  RodConstraint(Particle *p1, Particle * p2, double dist, bool useSqrt);

  double C() const override;
  double C_dot() const override;
  bool m_useSqrt = false;
  std::vector<Particle*> getParticles() const override;
  std::vector<Vec2f> J_rows() const override;
  std::vector<Vec2f> J_dot_rows() const override;
  void draw() override;

 private:
  Particle * const m_p1;
  Particle * const m_p2;
  double const m_dist;
};