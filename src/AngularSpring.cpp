#include "AngularSpring.h"
#include "GLCompat.h"
#include <cmath>

AngularSpring::AngularSpring(Particle *a, Particle *b, Particle *c,
                             double rest_angle, double ks, double kd) :
    m_a(a), m_b(b), m_c(c),
    m_rest_angle(rest_angle), m_ks(ks), m_kd(kd) {}

void AngularSpring::apply() {
  // Vectors from joint B out to A and C
  Vec2f u = m_a->m_Position - m_b->m_Position;
  Vec2f v = m_c->m_Position - m_b->m_Position;

  // Squared lengths
  float u_len_sq = u * u;
  float v_len_sq = v * v;
  // Avoid arm blowup
  if (u_len_sq < 1e-4f || v_len_sq < 1e-4f) return;

  // 2D scalar cross and dot
  float cross = u[0] * v[1] - u[1] * v[0];
  float dot   = u[0] * v[0] + u[1] * v[1];

  // Current signed angle in (-pi, pi]
  float theta = atan2f(cross, dot);

  // Angle error, normalized to (-pi, pi] to avoid wrap-around spike at rest_angle = pi
  float dtheta = theta - (float)m_rest_angle;
  // Wrap to (-pi, pi] to handle the atan2 sign flip at +-pi
  while (dtheta >  M_PI) dtheta -= 2.0f * (float)M_PI;
  while (dtheta < -M_PI) dtheta += 2.0f * (float)M_PI;

  // Perpendiculars (rotate 90 deg CCW)
  Vec2f u_perp(-u[1], u[0]);
  Vec2f v_perp(-v[1], v[0]);

  // Spring forces from angle deviation
  Vec2f F_a =  float(m_ks) * dtheta * (u_perp / u_len_sq);
  Vec2f F_c = -float(m_ks) * dtheta * (v_perp / v_len_sq);

  // Damping: oppose angular velocity
  float u_len = sqrtf(u_len_sq);
  float v_len = sqrtf(v_len_sq);
  Vec2f u_perp_hat = u_perp / u_len;
  Vec2f v_perp_hat = v_perp / v_len;
  float ang_vel_a = ((m_a->m_Velocity - m_b->m_Velocity) * u_perp_hat) / u_len;
  float ang_vel_c = ((m_c->m_Velocity - m_b->m_Velocity) * v_perp_hat) / v_len;
  float dtheta_dot = ang_vel_c - ang_vel_a;

  F_a +=  float(m_kd) * dtheta_dot * (u_perp / u_len_sq) * 0.5f;
  F_c += -float(m_kd) * dtheta_dot * (v_perp / v_len_sq) * 0.5f;

  // Clamp to prevent impulse blow-up under large deformation
  const float kMaxForce = 50.0f;
  auto clamp2 = [&](Vec2f f) -> Vec2f {
    float mag2 = f * f;
    if (mag2 > kMaxForce * kMaxForce)
      return f * (kMaxForce / sqrtf(mag2));
    return f;
  };
  F_a = clamp2(F_a);
  F_c = clamp2(F_c);

  // Apply conserving momentum
  m_a->m_Force += F_a;
  m_c->m_Force += F_c;
  m_b->m_Force -= (F_a + F_c);
}

void AngularSpring::draw() {
  // Light orange connecting lines through the joint
  glBegin(GL_LINES);
  glColor3f(0.9f, 0.6f, 0.2f);
  glVertex2f(m_a->m_Position[0], m_a->m_Position[1]);
  glVertex2f(m_b->m_Position[0], m_b->m_Position[1]);
  glVertex2f(m_b->m_Position[0], m_b->m_Position[1]);
  glVertex2f(m_c->m_Position[0], m_c->m_Position[1]);
  glEnd();
}