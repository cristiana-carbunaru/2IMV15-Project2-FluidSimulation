#include "Particle.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "Force.h"
#include "Constraint.h"
#include "ConstraintSolver.h"
#include "linearSolver.h"


#include <vector>
#include <cmath>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)

extern int solver_type;
extern float dt;

/* get state from particle into dst */
void ParticleGetState(std::vector<Particle*> pVector, std::vector<float> &dst) {
	// clear dst
	dst.clear();
	
	int ii, size = pVector.size();

	for(ii=0; ii<size; ii++)
	{
		dst.push_back(pVector[ii]->m_Position[0]);
		dst.push_back(pVector[ii]->m_Position[1]);
		dst.push_back(pVector[ii]->m_Velocity[0]);
		dst.push_back(pVector[ii]->m_Velocity[1]);
	}
}

/* set state from src into particle */
void ParticleSetState(std::vector<Particle*> pVector, const std::vector<float> &src) {
	int ii, size = pVector.size();
	int ind = 0;

	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Position[0] = src[ind++];
		pVector[ii]->m_Position[1] = src[ind++];
		pVector[ii]->m_Velocity[0] = src[ind++];
		pVector[ii]->m_Velocity[1] = src[ind++];
	}
}

/* calculate derivative, place in dst */
void ParticleDerivative(std::vector<Particle*> pVector, 
	std::vector<Force*> fVector, 
	std::vector<Constraint*> cVector,
	std::vector<float> &dst)
{
	int ii, size = pVector.size();
	dst.clear();
	
	// zero the force accumulation
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Force = Vec2f(0.0, 0.0);
	}

	for (auto* f : fVector) {
		if (f->isEnabled()) f->apply();
	}

	// apply constraints
	solve_constraints(pVector, cVector);

	// fill dst with derivatives
	for(ii=0; ii<size; ii++)
	{
		// xdot = v
		dst.push_back(pVector[ii]->m_Velocity[0]);
		dst.push_back(pVector[ii]->m_Velocity[1]);
		// vdot = f/m
		dst.push_back(pVector[ii]->m_Force[0] / pVector[ii]->m_Mass);
		dst.push_back(pVector[ii]->m_Force[1] / pVector[ii]->m_Mass);
	}
	
}

// Euler solver
void euler_step(std::vector<Particle*> pVector, 
	std::vector<Force*> fVector, 
	std::vector<Constraint*> cVector,
	float deltaT) 
{
	std::vector<float> state, derivative;
	// get state
	ParticleGetState(pVector, state);
	// get derivative 
	ParticleDerivative(pVector, fVector, cVector, derivative);

	// Euler method: x_new = x + deltaT * xdot
	for (size_t ii = 0; ii < state.size(); ++ii) {
		state[ii] += derivative[ii] * deltaT;
	}

	ParticleSetState(pVector, state);
}

// Midpoint solver
void midpoint_step(std::vector<Particle*> pVector, 
	std::vector<Force*> fVector, 
	std::vector<Constraint*> cVector,
	float deltaT) 
{
	std::vector<float> state, derivative, midState, midDerivative;
	// get state
	ParticleGetState(pVector, state);
	// get derivative
	ParticleDerivative(pVector, fVector, cVector, derivative);
	
	// Euler step
	midState.resize(state.size());
	for (size_t ii = 0; ii < state.size(); ++ii) {
		midState[ii] = state[ii] + 0.5f * deltaT * derivative[ii];
	}
	
	// set particles to midpoint state
	ParticleSetState(pVector, midState);
	
	// evaluate f at the midpoint
	ParticleDerivative(pVector, fVector, cVector, midDerivative);
	
	// take a step using the midpoint value
	for (size_t ii = 0; ii < state.size(); ++ii) {
		state[ii] += midDerivative[ii] * deltaT;
	}
	ParticleSetState(pVector, state);
}

// RK4 solver
void rk4_step(std::vector<Particle*> pVector, 
	std::vector<Force*> fVector,
	std::vector<Constraint*> cVector,
	float deltaT) 
{
	std::vector<float> state, k1, k2, k3, k4;
	// get state
	ParticleGetState(pVector, state);
	
	// k1 = f(x)
	ParticleDerivative(pVector, fVector, cVector, k1);
	
	// k2 = f(x + 0.5*dt*k1)
	std::vector<float> tempState(state.size());
	for (size_t ii = 0; ii < state.size(); ++ii) {
		tempState[ii] = state[ii] + 0.5f * deltaT * k1[ii];
	}
	ParticleSetState(pVector, tempState);
	ParticleDerivative(pVector, fVector, cVector, k2);
	
	// k3 = f(x + 0.5*dt*k2)
	for (size_t ii = 0; ii < state.size(); ++ii) {
		tempState[ii] = state[ii] + 0.5f * deltaT * k2[ii];
	}
	ParticleSetState(pVector, tempState);
	ParticleDerivative(pVector, fVector, cVector, k3);
	
	// k4 = f(x + dt*k3)
	for (size_t ii = 0; ii < state.size(); ++ii) {
		tempState[ii] = state[ii] + deltaT * k3[ii];
	}
	ParticleSetState(pVector, tempState);
	ParticleDerivative(pVector, fVector, cVector, k4);
	
	// Runge-Kutta of order 4: x_new = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
	for (size_t ii = 0; ii < state.size(); ++ii) {
		state[ii] += (deltaT / 6.0f) * (k1[ii] + 2.0f * k2[ii] + 2.0f * k3[ii] + k4[ii]);
	}
	ParticleSetState(pVector, state);
}

class SystemMatrix : public implicitMatrix {
public:
    std::vector<Particle*>& pVector;
    std::vector<Force*>& fVector;
    float dt;

    SystemMatrix(std::vector<Particle*>& p, std::vector<Force*>& f, float delta_t) 
        : pVector(p), fVector(f), dt(delta_t) {}

    void matVecMult(double x[], double r[]) override {
        int N = pVector.size();
        std::vector<double> dx(N * 2);
        std::vector<double> df(N * 2, 0.0);

        for (int i = 0; i < N * 2; ++i) {
            dx[i] = x[i] * dt;
        }

        for (Force* f : fVector) {
            if (f->isEnabled()) f->addJacobianMultiplication(dx.data(), x, df.data(), N);
        }

        for (int i = 0; i < N; ++i) {
            if (pVector[i]->m_Pinned) {
                r[i*2] = x[i*2];
                r[i*2+1] = x[i*2+1];
            } else {
                float mass = pVector[i]->m_Mass; 
                r[i*2]   = mass * x[i*2]   - dt * df[i*2];
                r[i*2+1] = mass * x[i*2+1] - dt * df[i*2+1];
            }
        }
    }
};

void implicit_euler(std::vector<Particle*>& pVector, std::vector<Force*>& fVector, std::vector<Constraint*>& cVector, float dt) {
    int N = pVector.size();
    int N2 = N * 2;
    
    std::vector<double> x(N2, 0.0); 
    std::vector<double> b(N2, 0.0); 

    for (Particle* p : pVector) p->clearForce();
    for (Force* f : fVector) if (f->isEnabled()) f->apply();
	solve_constraints(pVector, cVector);

    std::vector<double> v0(N2);
    std::vector<double> zero_dv(N2, 0.0);
    std::vector<double> df_dx_v0(N2, 0.0);

    for (int i = 0; i < N; ++i) {
        v0[i*2] = pVector[i]->m_Velocity[0];
        v0[i*2+1] = pVector[i]->m_Velocity[1];
    }

    for (Force* f : fVector) {
        if (f->isEnabled()) f->addJacobianMultiplication(v0.data(), zero_dv.data(), df_dx_v0.data(), N);
    }

    for (int i = 0; i < N; ++i) {
        if (pVector[i]->m_Pinned) {
            b[i*2] = 0.0;
            b[i*2+1] = 0.0;
        } else {
            b[i*2]   = dt * (pVector[i]->m_Force[0] + dt * df_dx_v0[i*2]);
            b[i*2+1] = dt * (pVector[i]->m_Force[1] + dt * df_dx_v0[i*2+1]);
        }
    }

    SystemMatrix A(pVector, fVector, dt);
    int steps = 0;
    ConjGrad(N2, &A, x.data(), b.data(), 1e-4, &steps);

    for (int i = 0; i < N; ++i) {
        if (!pVector[i]->m_Pinned) {
            pVector[i]->m_Velocity[0] += x[i*2];
            pVector[i]->m_Velocity[1] += x[i*2+1];
            pVector[i]->m_Position[0] += dt * pVector[i]->m_Velocity[0];
            pVector[i]->m_Position[1] += dt * pVector[i]->m_Velocity[1];
        }
    }

	for (Particle* p : pVector) p->clearForce();
	for (Force* f : fVector) if (f->isEnabled()) f->apply();
	solve_constraints(pVector, cVector);
}

void verlet_step(std::vector<Particle*>& pVector, std::vector<Force*>& fVector, std::vector<Constraint*>& cVector, float dt) {
    int N = pVector.size();
    std::vector<Vec2f> old_forces(N, Vec2f(0.0f, 0.0f));

    for (Particle* p : pVector) p->clearForce();
    for (Force* f : fVector) if (f->isEnabled()) f->apply();
	solve_constraints(pVector, cVector);

    for (int i = 0; i < N; ++i) {
        Particle* p = pVector[i];
        old_forces[i] = p->m_Force;

        if (!p->m_Pinned) {
            p->m_Position[0] += p->m_Velocity[0] * dt + 0.5f * (p->m_Force[0] / p->m_Mass) * (dt * dt);
            p->m_Position[1] += p->m_Velocity[1] * dt + 0.5f * (p->m_Force[1] / p->m_Mass) * (dt * dt);
        }
    }

    for (Particle* p : pVector) p->clearForce();
    for (Force* f : fVector) if (f->isEnabled()) f->apply();
	solve_constraints(pVector, cVector);

    for (int i = 0; i < N; ++i) {
        Particle* p = pVector[i];
        if (!p->m_Pinned) {
            p->m_Velocity[0] += 0.5f * ((old_forces[i][0] + p->m_Force[0]) / p->m_Mass) * dt;
            p->m_Velocity[1] += 0.5f * ((old_forces[i][1] + p->m_Force[1]) / p->m_Mass) * dt;
        }
    }
	solve_constraints(pVector, cVector);
}

extern void simulation_step( std::vector<Particle*> pVector, 
	std::vector<Force*> fVector, 
	std::vector<Constraint*> cVector, 
	float dt )
{
	switch(solver_type)
	{
		case 0: euler_step(pVector, fVector, cVector, dt); break;
		case 1: midpoint_step(pVector, fVector, cVector, dt); break;
		case 2: rk4_step(pVector, fVector, cVector, dt); break;
		case 3: implicit_euler(pVector, fVector, cVector, dt); break;
		case 4: verlet_step(pVector, fVector, cVector, dt); break;
	}
}
