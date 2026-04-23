#pragma once
#include "glm/vec2.hpp"
#include "vector"

namespace physics
{
// Variable to indicate walls, walls have the same id in collidedId.
const entt::entity WALLSID = static_cast<entt::entity>(UINT32_MAX);

struct CollisionData
{
    glm::vec2 normal;
    float depth;
    entt::entity collidedId;
    float inverseMassOfSecondBody = 1 / 1.0f;
};

enum class BodyType
{
    STATIC = 0,
    KINEMATIC = 1,
    DYNAMIC = 2,
};

struct DiskColliderComponent
{
    float r;
    DiskColliderComponent(const float& _r = 0) : r(_r)
    {
        // Preallocate space for 16 collisions to reduce dynamic memory allocation during collision detection
        collisionData.reserve(16);
    }
    std::vector<CollisionData> collisionData;
};

struct BodyComponent
{
    BodyComponent(const glm::vec2& _possition = glm::vec2(0, 0),
                  const BodyType& _bodyType = BodyType::DYNAMIC,
                  const bool& _applyGravity = true,
                  const float& _mass = 1.0f,
                  const float& _e = 0.4f)
        : possition(_possition), inverseMass(1.0f / _mass), e(_e), applyGravity(_applyGravity), bodyType(_bodyType)
    {
    }
    glm::vec2 possition;
    glm::vec2 velocity = {0, 0};
    float inverseMass;
    float e;
    bool applyGravity = true;
    BodyType bodyType;
};
}  // namespace physics
