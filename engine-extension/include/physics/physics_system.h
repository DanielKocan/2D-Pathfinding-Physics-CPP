#pragma once
#include "core/ecs.hpp"
#include "core/engine.hpp"
#include "core/transform.hpp"
#include "glm/vec2.hpp"
#include "physics/physics_components.h"
#include "core/geometry2d.hpp"

using namespace bee;
using namespace geometry2d;

namespace physics
{

class physics_system : public bee::System
{
public:
    physics_system(const bool& _gravityMode,
                   const PolygonList& _polygonsFromMap,
                   const float& fixedFPS = 50.0f,
                   const int& _collisionResolutionIterations = 2)
        : polygonsFromMap(_polygonsFromMap),
          fixedTimeStep(1.0f / fixedFPS),
          collisionResolutionIterations(_collisionResolutionIterations)
    {
        gravityMode = _gravityMode;
    }
    void Update(float deltaTime) override;
    void Render() override;

    static void ApplyImpulse(BodyComponent& Body, const glm::vec2& impulse);
    static void ApplyImpulse(BodyComponent& Body, const glm::vec2& direction, const float& j);

    inline static bool HasExecutedFrame() { return hasExecutedFrame; }

    static bool gravityMode;

private:
    void CollisionDetection();
    static void ResolveOverlap();

    void PrepareCollisionInterfaceData();
    void ApplyCollisionDataInterface();

    static void ClearAllCollisionData();

    static bool hasExecutedFrame;
    float timeAccumulator = 0.0f;  // Stores unprocessed time
    PolygonList polygonsFromMap;
    const float GRAVITY = 9.81f;
    float fixedTimeStep = 1.0f / 50.0f;  // 50 updates per second
    int collisionResolutionIterations = 2;

    std::unordered_map<entt::entity, std::vector<physics::CollisionData>> tempCollisionData;
};

}  // namespace physics
