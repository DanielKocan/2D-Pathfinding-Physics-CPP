#pragma once
#include "core/ecs.hpp"
#include "core/engine.hpp"
#include "ai/agent_components.h"
#include "ai/NavigationMesh.h"
#include "ai/map.h"
#include "physics/physics_components.h"
#include "graph.h"
#include "../../game/include/player_input_system.h"

namespace bee
{

class navigation_system : public bee::System
{
public:
    // Navigation system needs bee::Navigation and physics::Body components to work.
    navigation_system(const std::shared_ptr<map>& _gameMap, const float& fixedFPS = 10.0f);
    void Update(float deltaTime) override;
    void Render() override;  // For debug circles

    static void CalculateNewPathForAllAgents(const glm::vec2& goalPos);
    static void CalculateNewPathForAgent(const entt::entity& agentEntity, const glm::vec2& goalPos);

    glm::vec2 getAttractionPoint(NavigationComponent navigation, glm::vec2 agentPos);

private:
    static std::shared_ptr<graph> pGameMapGraph;
    std::shared_ptr<NavigationMesh> pNavMesh;

    float fixedTimeStep = 1.0f / 10.0f;  // 10 updates per second
    float timeAccumulator = 0.0f;        // Stores unprocessed time

    static int getRefIndexOfClosestPoint(NavigationComponent navigation, glm::vec2 agentPos);
    inline bool ArePointsApproximatelyEqual(glm::vec2 point1, glm::vec2 point2, float epsilon)
    {
        float distance = glm::distance(point1, point2);
        return distance <= epsilon;
    }
};

}  // namespace bee
