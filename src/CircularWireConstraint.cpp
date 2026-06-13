#include "CircularWireConstraint.h"
#include "GLCompat.h"

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2f &vect, float radius)
{
	// no draw for small radius
	if (radius < 1e-6) return;

	glBegin(GL_LINE_LOOP);
	glColor3f(0.0, 1.0, 0.0);
	for (int i = 0; i < 360; i = i + 18)
	{
		float degInRad = i * PI / 180;
		glVertex2f(vect[0] + cos(degInRad) * radius, vect[1] + sin(degInRad) * radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec2f &center, const double radius) : 
m_p(p), m_center(center), m_radius(radius) {}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

double CircularWireConstraint::C() const
{
	Vec2f d = m_p->m_Position - m_center;
	return d * d - m_radius * m_radius; // Vec2f's operator* is dot product 
}

double CircularWireConstraint::C_dot() const
{
	Vec2f d = m_p->m_Position - m_center;
	return 2.0 * (d * m_p->m_Velocity);
}

std::vector<Particle *> CircularWireConstraint::getParticles() const
{
	return {m_p};
}

std::vector<Vec2f> CircularWireConstraint::J_rows() const
{
	Vec2f d = m_p->m_Position - m_center;
	return {2.0f * d};
}

std::vector<Vec2f> CircularWireConstraint::J_dot_rows() const
{
	return {2.0f * m_p->m_Velocity};
}
