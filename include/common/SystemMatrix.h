#pragma once

#include <vector>
#include "Particle.h"
#include "Force.h"
#include "linearSolver.h"

class SystemMatrix : public implicitMatrix {
public:
    SystemMatrix(std::vector<Particle*>& p, std::vector<Force*>& f, float delta_t);
    void matVecMult(double x[], double r[]) override;

private:
    std::vector<Particle*>& pVector;
    std::vector<Force*>& fVector;
    float dt;
};
