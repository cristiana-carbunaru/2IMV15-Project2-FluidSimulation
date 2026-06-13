#ifndef CONSTRAINT_SOLVER_H
#define CONSTRAINT_SOLVER_H

#include "Particle.h"
#include "Constraint.h"
#include <vector>


void solve_constraints(std::vector<Particle*>& particles,
                       std::vector<Constraint*>& constraints);

#endif