#pragma once

#include "Particle.h"
#include "Force.h"

class SpringForce : public Force {
 public:
  SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd);

  void apply() override;
  void draw() override;

  Particle* getP1()   const { return m_p1;   }
  Particle* getP2()   const { return m_p2;   }
  double    getDist() const { return m_dist;  }
  double    getKs()   const { return m_ks;   }
  double    getKd()   const { return m_kd;   }

 private:

  Particle * const m_p1;   // particle 1
  Particle * const m_p2;   // particle 2
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
