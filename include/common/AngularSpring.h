#pragma once

#include "Force.h"
#include "Particle.h"

class AngularSpring : public Force {
 public:
  AngularSpring(Particle *a, Particle *b, Particle *c,
                double rest_angle, double ks, double kd);

  void apply() override;
  void draw() override;

 private:
  Particle * const m_a;   // outer particle 1
  Particle * const m_b;   // joint (middle)
  Particle * const m_c;   // outer particle 2
  double const m_rest_angle;
  double const m_ks, m_kd;
};