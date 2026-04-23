#pragma once
#include "glm/vec2.hpp"
namespace game
{
struct PlayerComponent
{
    float movementSpeed = 1000.0f;
};

struct BulletComponent
{
    glm::vec2 direction;
    const float movingSpeed = 10.0f;
    int numOfBounces;
    float r = 0.2f;
    BulletComponent(glm::vec2 _direction, int _numOfBounces = 2)
        : direction(glm::normalize(_direction)), numOfBounces(_numOfBounces)
    {
    }
};
}  // namespace game
