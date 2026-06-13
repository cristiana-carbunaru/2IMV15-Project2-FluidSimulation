#include "RodConstraint.h"
#include "GLCompat.h"

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist, bool useSqrt)
    : m_useSqrt(useSqrt), m_p1(p1), m_p2(p2), m_dist(dist) {}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.55, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.55, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}

double RodConstraint::C() const
{
  Vec2f d = m_p1->m_Position - m_p2->m_Position;
  double c_val = m_useSqrt ? sqrt(d * d) - m_dist : d * d - m_dist * m_dist;
  return c_val;
}

double RodConstraint::C_dot() const
{
  Vec2f d = m_p1->m_Position - m_p2->m_Position;
  Vec2f v_rel = m_p1->m_Velocity - m_p2->m_Velocity;
  if (d * d < 1e-6) {
    return 0.0;
  }
  // sqrt vs non-sqrt version
  return m_useSqrt ? (d * v_rel) / sqrt(d * d) : 2.0 * (d * v_rel);
}

std::vector<Particle *> RodConstraint::getParticles() const
{
  return {m_p1, m_p2};
}

std::vector<Vec2f> RodConstraint::J_rows() const
{
  Vec2f d = m_p1->m_Position - m_p2->m_Position;
  if (d * d < 1e-6) {
    return {Vec2f(1.0, 0.0), Vec2f(-1.0, 0.0)};
  }
  // sqrt vs non-sqrt version
  Vec2f J_rows = m_useSqrt ? d / sqrt(d * d) : 2.0f * d;
  return {J_rows, -J_rows};
}

std::vector<Vec2f> RodConstraint::J_dot_rows() const
{
  Vec2f d = m_p1->m_Position - m_p2->m_Position;
  Vec2f v = m_p1->m_Velocity - m_p2->m_Velocity;
  double magnitude = sqrt(d * d);
  if (magnitude < 1e-6) return { Vec2f(0.0, 0.0), Vec2f(0.0, 0.0) };
  // sqrt vs non-sqrt version
  Vec2f J_dot_rows = m_useSqrt ? (v / sqrt(d * d)) - (d * (d * v) / pow(d * d, 1.5)) : 2.0f * v;
  return {J_dot_rows, -J_dot_rows};
}