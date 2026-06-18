#include "ConstraintSolver.h"
#include "linearSolver.h"
#include <vector>
#include <cstddef>

static const double K_S = 30.0;  // spring-like correction on C
static const double K_D = 10.0;  // damper-like correction on C_dot

static int particle_index(Particle* p, const std::vector<Particle*>& particles)
{
    for (std::size_t i = 0; i < particles.size(); i++) {
        if (particles[i] == p) return (int)i;
    }
    return -1;
}

// Implicit representation of the matrix A = J W J^T
class JWJTMatrix : public implicitMatrix
{
public:
    std::vector<Particle*>*   particles;
    std::vector<Constraint*>* constraints;

    void matVecMult(double x[], double r[]) override
    {
        const int n = (int)particles->size();
        const int m = (int)constraints->size();

        std::vector<Vec2f> y1(n, Vec2f(0.0f, 0.0f));
        for (int i = 0; i < m; i++) {
            Constraint* c = (*constraints)[i];
            std::vector<Particle*> cp = c->getParticles();
            std::vector<Vec2f> rows = c->J_rows();
            for (std::size_t k = 0; k < cp.size(); k++) {
                int p_idx = particle_index(cp[k], *particles);
                y1[p_idx] += float(x[i]) * rows[k];
            }
        }

        std::vector<Vec2f> y2(n);
        for (int p = 0; p < n; p++) {
            y2[p] = y1[p] / (*particles)[p]->m_Mass;
        }

        for (int i = 0; i < m; i++) {
            Constraint* c = (*constraints)[i];
            std::vector<Particle*> cp = c->getParticles();
            std::vector<Vec2f> rows = c->J_rows();
            double sum = 0.0;
            for (std::size_t k = 0; k < cp.size(); k++) {
                int p_idx = particle_index(cp[k], *particles);
                sum += rows[k] * y2[p_idx];   // Vec2f * Vec2f = dot product
            }
            r[i] = sum;
        }
    }
};

void solve_constraints(std::vector<Particle*>& particles,
                       std::vector<Constraint*>& constraints)
{
    std::vector<Constraint*> active_constraints;
    for (auto* c : constraints) {
        if (c->isEnabled()) active_constraints.push_back(c);
    }
    const int m = (int)active_constraints.size();
    if (m == 0) return;

    std::vector<double> b(m, 0.0);
    for (int i = 0; i < m; i++) {
        Constraint* c = active_constraints[i];
        std::vector<Particle*> cp = c->getParticles();
        std::vector<Vec2f> J_rows     = c->J_rows();
        std::vector<Vec2f> J_dot_rows = c->J_dot_rows();

        double jdot_xdot = 0.0;   // J_dot * x_dot for this constraint row
        double jwq       = 0.0;   // J W Q for this constraint row

        for (std::size_t k = 0; k < cp.size(); k++) {
            Particle* p = cp[k];
            jdot_xdot += J_dot_rows[k] * p->m_Velocity;
            jwq       += J_rows[k]     * (p->m_Force / p->m_Mass);
        }

        const double Ci     = c->C();
        const double Ci_dot = c->C_dot();
        b[i] = -jdot_xdot - jwq - K_S * Ci - K_D * Ci_dot;
    }

    // Set up the implicit matrix A = J W J^T
    JWJTMatrix A;
    A.particles   = &particles;
    A.constraints = &active_constraints;

    // Solve A * lambda = b with conjugate gradient
    std::vector<double> lambda(m, 0.0);
    int steps = 0;
    const double epsilon = 1e-6;
    ConjGrad(m, &A, lambda.data(), b.data(), epsilon, &steps);

    for (int i = 0; i < m; i++) {
        Constraint* c = active_constraints[i];
        std::vector<Particle*> cp = c->getParticles();
        std::vector<Vec2f> rows = c->J_rows();
        for (std::size_t k = 0; k < cp.size(); k++) {
            cp[k]->m_Force += float(lambda[i]) * rows[k];
        }
    }
}