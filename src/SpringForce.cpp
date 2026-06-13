#include "SpringForce.h"
#include "GLCompat.h"

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

void SpringForce::apply() {
  // Calculate vector between particles: l = p1->pos - p2->pos
  Vec2f l = m_p1->m_Position - m_p2->m_Position;

  // Calculate distance
  float length = sqrt(l[0]*l[0] + l[1]*l[1]);
  if (length < 1e-6) return; // avoid division by zero

  // Calculate relative velocity: v = p1->vel - p2->vel
  Vec2f v_rel = m_p1->m_Velocity - m_p2->m_Velocity;

  // Normalize l to get the direction of the force
  Vec2f l_dir = l / length;

  // Hooke's Law + Damping: F = -[ks * (|l| - rest_dist) + kd * ((v1-v2) dot (l / |l|))] * (l / |l|)
  float force_magnitude = -(m_ks * (length - m_dist) + m_kd * (v_rel * l_dir));
  Vec2f f = force_magnitude * l_dir;

  // Apply equal and opposite forces (Newton's 3rd Law)
  m_p1->m_Force += f;
  m_p2->m_Force -= f;
}