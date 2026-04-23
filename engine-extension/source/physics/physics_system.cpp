#include "physics/physics_system.h"
#include "ai/agent_components.h"
#include "iostream"  // for cout
#include "rendering/debug_render.hpp"

/*
1.Add forces to physics bodies (e.g. gravity)
2.Update physics bodies (first update velocity using F = ma; then position)
3.Detect + resolve collisions (Week 5)
4.Clear forces for next frame
5.Sync transforms with physics body positions
*/

bool physics::physics_system::gravityMode = true;
bool physics::physics_system::hasExecutedFrame = false;

void physics::physics_system::Update(float deltaTime)
{
    hasExecutedFrame = false;

    ClearAllCollisionData();

    // Accumulator for fixed time steps
    timeAccumulator += deltaTime;

    // Fixed time steps (Should be 50 fps by default)
    while (timeAccumulator >= fixedTimeStep)
    {
        // Physics logic for Ai moving agents
        auto agentNavigationView = bee::Engine.ECS().Registry.view<physics::BodyComponent, const bee::NavigationComponent>();
        for (auto [entity, Body, Navigation] : agentNavigationView.each())
        {
            if (Body.bodyType == BodyType::STATIC || Body.bodyType == BodyType::KINEMATIC) continue;

            if (gravityMode) break;  // We dont want to do AI movement logic if they have gravity mode

            if (Navigation.attractionPoint != glm::vec2(0, 0) && !Navigation.Path.empty())
            {
                glm::vec2 direction = Navigation.attractionPoint - Body.possition;
                direction = glm::normalize(direction);
                Body.velocity = direction * Navigation.walkingSpeed;
            }
            else
            {
                Body.velocity = glm::vec2(0.0f);
            }
        }

        // Apply gravity for phyics bodies
        auto agentView = bee::Engine.ECS().Registry.view<physics::BodyComponent>();
        for (auto [entity, Body] : agentView.each())
        {
            if (Body.bodyType == BodyType::STATIC) continue;

            if (gravityMode && Body.applyGravity) Body.velocity.y -= GRAVITY * fixedTimeStep;

            Body.possition += Body.velocity * fixedTimeStep;
        }

        for (int i = 0; i < collisionResolutionIterations; i++)
        {
            CollisionDetection();
            if (i == 0) PrepareCollisionInterfaceData();
            ResolveOverlap();
        }

        ApplyCollisionDataInterface();

        hasExecutedFrame = true;
        timeAccumulator -= fixedTimeStep;
    }
}

void physics::physics_system::Render()
{
    // Agent movement to attracttion point created by navigationSystem
    auto agentView =
        bee::Engine.ECS().Registry.view<bee::Transform, const bee::NavigationComponent, const physics::BodyComponent>();
    auto bodiesView = bee::Engine.ECS().Registry.view<bee::Transform, const physics::BodyComponent>();

    for (auto [entity, Transform, Navigation, Body] : agentView.each())
    {
        bee::Engine.DebugRenderer().AddCircle(bee::DebugCategory::Gameplay,
                                              glm::vec3(Navigation.attractionPoint, 0),
                                              0.15f,
                                              glm::vec4(1, 1, 1, 1));
    }

    for (auto [entity, Transform, Body] : bodiesView.each())
    {
        Transform.SetTranslation(glm::vec3(Body.possition.x, Body.possition.y, 0));
    }
}

void physics::physics_system::ApplyImpulse(BodyComponent& Body, const glm::vec2& impulse)
{
    if (Body.inverseMass == 0.0f) return;
    Body.velocity += impulse * Body.inverseMass;
}

void physics::physics_system::ApplyImpulse(BodyComponent& Body, const glm::vec2& direction, const float& j)
{
    if (Body.inverseMass == 0.0f) return;
    Body.velocity += glm::normalize(direction) * j * Body.inverseMass;
}

void physics::physics_system::CollisionDetection()
{
    auto diskColliderView = bee::Engine.ECS().Registry.view<physics::DiskColliderComponent, physics::BodyComponent>();

    for (auto entityIt = diskColliderView.begin(); entityIt != diskColliderView.end(); ++entityIt)
    {
        auto entity = *entityIt;
        auto& DiskCollider = diskColliderView.get<physics::DiskColliderComponent>(entity);
        auto& Body = diskColliderView.get<physics::BodyComponent>(entity);

        if (Body.bodyType == BodyType::STATIC || Body.bodyType == BodyType::KINEMATIC) Body.inverseMass = 0.0f;

        for (auto otherEntityIt = std::next(entityIt); otherEntityIt != diskColliderView.end(); ++otherEntityIt)
        {
            auto entity2 = *otherEntityIt;
            auto& DiskCollider2 = diskColliderView.get<physics::DiskColliderComponent>(entity2);
            auto& Body2 = diskColliderView.get<physics::BodyComponent>(entity2);

            if (Body2.bodyType == BodyType::STATIC || Body2.bodyType == BodyType::KINEMATIC) Body2.inverseMass = 0.0f;

            float distance = glm::distance(Body.possition, Body2.possition);
            float rSum = DiskCollider.r + DiskCollider2.r;

            // Check for collision between the two entities
            if (distance <= rSum)
            {
                CollisionData newCollData;
                glm::vec2 VectorBetweenCenters = Body.possition - Body2.possition;

                newCollData.inverseMassOfSecondBody = Body2.inverseMass;
                newCollData.normal = normalize(VectorBetweenCenters);
                newCollData.depth = rSum - distance;
                newCollData.collidedId = entity2;

                DiskCollider.collisionData.push_back(newCollData);
            }
        }

        for (const Polygon& polygon : polygonsFromMap)
        {
            // Check if the circle's center is inside the polygon
            bool isInside = geometry2d::IsPointInsidePolygon(Body.possition, polygon);

            if (isInside == false)
            {
                glm::vec2 closestPointOnPolygon = geometry2d::GetNearestPointOnPolygonBoundary(Body.possition, polygon);
                float distance = glm::distance(Body.possition, closestPointOnPolygon);
                if (distance <= DiskCollider.r)
                {
                    CollisionData newCollWithPolygon;
                    newCollWithPolygon.inverseMassOfSecondBody = 0.0f;
                    newCollWithPolygon.normal = glm::normalize(Body.possition - closestPointOnPolygon);
                    newCollWithPolygon.depth = DiskCollider.r - distance;
                    newCollWithPolygon.collidedId = static_cast<entt::entity>(UINT32_MAX);

                    DiskCollider.collisionData.push_back(newCollWithPolygon);
                }
            }
            else
            {
                glm::vec2 closestPointOnPolygon = geometry2d::GetNearestPointOnPolygonBoundary(Body.possition, polygon);
                float distance = glm::distance(Body.possition, closestPointOnPolygon);

                CollisionData newCollWithPolygon;
                newCollWithPolygon.inverseMassOfSecondBody = 0.0f;
                newCollWithPolygon.normal = glm::normalize(Body.possition + closestPointOnPolygon);
                newCollWithPolygon.depth = DiskCollider.r + distance;
                newCollWithPolygon.collidedId = static_cast<entt::entity>(UINT32_MAX);

                DiskCollider.collisionData.push_back(newCollWithPolygon);
            }
        }
    }
}

void physics::physics_system::ResolveOverlap()
{
    auto agentNavigationView = bee::Engine.ECS().Registry.view<physics::BodyComponent, physics::DiskColliderComponent>();

    for (auto [entity, Body, DiskCollider] : agentNavigationView.each())
    {
        for (const auto& collision : DiskCollider.collisionData)
        {
            float totalInverseMass = Body.inverseMass + collision.inverseMassOfSecondBody;
            if (totalInverseMass == 0.0f) continue;  // Both bodies are static, skip

            if (collision.inverseMassOfSecondBody == 0.0f)  // collision with polygon (static object)
            {
                Body.possition += collision.normal * (collision.depth * 1.0f);

                // Calculate velocity along the normal direction
                float velocityAlongNormal = glm::dot(Body.velocity, collision.normal);

                // Skip resolving if the objects are separating
                if (velocityAlongNormal >= 0)
                    continue;  // This indicates that the angle between the two vectors is smaller than 90 degrees, meaning they
                               // are pointing in generally same directions.
                float e = Body.e;
                float impulseMagnitude = -(1.0f + e) * velocityAlongNormal / Body.inverseMass;
                Body.velocity += impulseMagnitude * collision.normal;
            }
            else  // coolliison with other disk (dynamic object)
            {
                // Move bodies apart based on their masses
                float bodyAPenetration = collision.depth * (Body.inverseMass / totalInverseMass);
                float bodyBPenetration = collision.depth * (collision.inverseMassOfSecondBody / totalInverseMass);

                auto& SecondBody = bee::Engine.ECS().Registry.get<physics::BodyComponent>(collision.collidedId);
                Body.possition += collision.normal * bodyAPenetration;
                SecondBody.possition -= collision.normal * bodyBPenetration;

                // Calculate relative velocity
                glm::vec2 relativeVelocity = Body.velocity - SecondBody.velocity;

                // Calculate velocity along the normal direction
                float velocityAlongNormal = glm::dot(relativeVelocity, collision.normal);
                // Skip resolving if the objects are separating
                if (velocityAlongNormal > 0) continue;

                // Calculate impulse magnitude
                float e = Body.e;  // Coefficient of restitution
                float j = -(1 + e) * velocityAlongNormal / totalInverseMass;

                // Apply impulse
                glm::vec2 impulse = j * collision.normal;
                Body.velocity += Body.inverseMass * impulse;
                SecondBody.velocity -= collision.inverseMassOfSecondBody * impulse;
            }
        }
    }

    for (auto [entity, Body, DiskCollider] : agentNavigationView.each())
    {
        // if (DiskCollider.collisionData.size() > test) test = static_cast<int>(DiskCollider.collisionData.size());
        DiskCollider.collisionData.clear();
    }
    // std::cout <<"size: "<< test << std::endl;
}

void physics::physics_system::PrepareCollisionInterfaceData()
{
    tempCollisionData.clear();  // Clear previous frame’s data

    auto BodyCollisionView =
        bee::Engine.ECS().Registry.view<const physics::BodyComponent, const physics::DiskColliderComponent>();
    for (auto [entity, Body, DiskCollider] : BodyCollisionView.each())
    {
        // Store the original collision data for the current entity
        for (const auto& collision : DiskCollider.collisionData)
        {
            tempCollisionData[entity].push_back(collision);
        }

        // For each collision, ensure the other entity is aware of the collision
        for (const auto& collision : DiskCollider.collisionData)
        {
            entt::entity secondEntity = collision.collidedId;
            CollisionData reverseCollision = collision;  // Copy the existing collision
            reverseCollision.collidedId = entity;        // Reverse collision ID to point back to the first entity
            tempCollisionData[secondEntity].push_back(reverseCollision);  // Add to the second entity
        }
    }
}

void physics::physics_system::ApplyCollisionDataInterface()
{
    auto BodyCollisionView = bee::Engine.ECS().Registry.view<const physics::BodyComponent, physics::DiskColliderComponent>();
    for (auto [entity, Body, DiskCollider] : BodyCollisionView.each())
    {
        if (tempCollisionData.find(entity) != tempCollisionData.end())
        {
            // Assign the saved collision data from tempCollisionData back to DiskCollider
            DiskCollider.collisionData = tempCollisionData[entity];
        }
    }
}

void physics::physics_system::ClearAllCollisionData()
{
    // Clear previous collision data for each DiskColliderComponent
    auto BodyCollisionView2 = bee::Engine.ECS().Registry.view<const physics::BodyComponent, physics::DiskColliderComponent>();
    for (auto [entity, Body, DiskCollider] : BodyCollisionView2.each())
    {
        DiskCollider.collisionData.clear();
    }
}
