#pragma once
#include <vector>
#include <cmath>
#include "Particle.h"

struct Wall {
    Vec2f a;
    Vec2f b;
    Vec2f normal;

    Wall(const Vec2f& a_, const Vec2f& b_)
        : a(a_), b(b_) {
        Vec2f edge = b - a;
        float length = sqrt(edge * edge);
        if (length > 1e-6f) {
            normal = Vec2f(-edge[1] / length, edge[0] / length);
        } else {
            normal = Vec2f(0.0f, 0.0f);
        }
    }

    void orientNormalToPoint(const Vec2f& point) {
        Vec2f edge = b - a;
        float length = sqrt(edge * edge);
        if (length < 1e-6f) return;
        Vec2f candidate(-edge[1] / length, edge[0] / length);
        if (((point - a) * candidate) < 0.0f) {
            candidate = -candidate;
        }
        normal = candidate;
    }
};

class CollisionHandler {
public:
    static void handleWallCollisions(std::vector<Particle*>& particles, const std::vector<Wall>& walls, float restitution, float friction);
    static void handleParticleCollisions(std::vector<Particle*>& particles, float particle_diameter, float restitution);
    // static void drawParticleCollisions(std::vector<Particle*>& particles, float particle_diameter);
    static void drawWalls(const std::vector<Wall>& walls);
};