#include "ai/navigation_system.h"

#include "core/engine.hpp"
#include "core/transform.hpp"
#include "rendering/debug_render.hpp"
#include "rendering/render.hpp"
#include <algorithm>

std::shared_ptr<graph> bee::navigation_system::pGameMapGraph = nullptr;

bee::navigation_system::navigation_system(const std::shared_ptr<map>& _gameMap, const float& fixedFPS)
{
    fixedTimeStep = 1.0f / fixedFPS;
    pNavMesh = std::make_shared<NavigationMesh>(_gameMap);
    pGameMapGraph = pNavMesh->getGraphPtr();
}

void bee::navigation_system::Update(float deltaTime)
{
    // Accumulator for fixed time steps
    timeAccumulator += deltaTime;

    // Fixed time steps (by default: 10 fps)
    while (timeAccumulator >= fixedTimeStep)
    {
        auto agentView = bee::Engine.ECS().Registry.view<bee::NavigationComponent, const physics::BodyComponent>();

        for (auto [entity, Navigation, Body] : agentView.each())
        {
            // Calculate attraction point (it depends on point and point+1)
            Navigation.attractionPoint = getAttractionPoint(Navigation, Body.possition);

            if (Navigation.attractionPoint == glm::vec2(0.0f)) Navigation.Path.clear();
        }
        timeAccumulator -= fixedTimeStep;
    }
}

void bee::navigation_system::Render()
{
    pNavMesh->drawGraph();
    pNavMesh->drawCDT();
    pNavMesh->drawPaths(pNavMesh->solution);
    pNavMesh->drawGraphPath();

    auto agentsView = bee::Engine.ECS().Registry.view<const bee::NavigationComponent, const physics::BodyComponent>();

    for (auto [entity, Navigation, Body] : agentsView.each())
    {
        bee::Engine.DebugRenderer().AddCircle(bee::DebugCategory::Gameplay,
                                              glm::vec3(Body.possition, 0),
                                              0.4f,
                                              glm::vec4(1, 0, 0, 1));
    }
}

int bee::navigation_system::getRefIndexOfClosestPoint(NavigationComponent navigation, glm::vec2 agentPos)
{
    int vertexIndex = pGameMapGraph->getClosestVertexIdFromPath(glm::vec3(agentPos, 0), navigation.Path);
    int ref = 0;

    for (size_t i = 0; i < navigation.Path.size(); i++)
    {
        if (navigation.Path[i] == vertexIndex)
        {
            ref = static_cast<int>(i);
            break;
        }
    }

    return ref;
}

glm::vec2 bee::navigation_system::getAttractionPoint(NavigationComponent navigation, glm::vec2 agentPos)
{
    float remainingDistance = navigation.lookAheadDistance;

    // Get current closest point (this part can be tricky, since we need index of vector which stores id of point)
    int currentIndex = getRefIndexOfClosestPoint(navigation, agentPos);

    if (ArePointsApproximatelyEqual(agentPos, navigation.finalPos, 0.1f))
    {
        // To stop chasing the point
        // navigation.attractionPoint = glm::vec2(0.0f);
        // navigation.Path.clear();
    }

    // Return if there was no path after Pathfinding
    if (navigation.Path.empty()) return glm::vec2(0, 0);

    if (currentIndex < static_cast<int>(navigation.Path.size()) - 1)  // To prevent beeing stucked bewtween two points
    {
        glm::vec2 nearestPoint =
            bee::geometry2d::GetNearestPointOnLineSegment(agentPos,
                                                          pGameMapGraph->vertices.at(navigation.Path[currentIndex]).pos,
                                                          pGameMapGraph->vertices.at(navigation.Path[currentIndex + 1]).pos);
        float offset = glm::distance(nearestPoint, glm::vec2(pGameMapGraph->vertices.at(navigation.Path[currentIndex]).pos));
        remainingDistance += offset;
    }

    // Move forward along the path until the desired look-ahead distance is covered
    while (currentIndex < static_cast<int>(navigation.Path.size()) - 1 && remainingDistance > 0)
    {
        int index = navigation.Path[currentIndex];
        int indexNext = navigation.Path[currentIndex + 1];

        float segmentLength = glm::distance(pGameMapGraph->vertices.at(index).pos, pGameMapGraph->vertices.at(indexNext).pos);

        if (segmentLength > remainingDistance)
        {
            // The attraction point is within this segment
            glm::vec2 direction =
                glm::normalize(pGameMapGraph->vertices.at(indexNext).pos - pGameMapGraph->vertices.at(index).pos);
            return pGameMapGraph->vertices.at(index).pos + glm::vec3((direction * remainingDistance), 0);
        }

        remainingDistance -= segmentLength;
        currentIndex++;
    }

    return navigation.finalPos;
}

void bee::navigation_system::CalculateNewPathForAllAgents(const glm::vec2& goalPos)
{
    auto agentView = bee::Engine.ECS().Registry.view<bee::NavigationComponent, const physics::BodyComponent>();
    for (auto [entity, Navigation, Body] : agentView.each())
    {
        int closestVertexIDToStart = pGameMapGraph->getClosestVertexId(glm::vec3(Body.possition, 0));
        int closestVertexIDToGoal = pGameMapGraph->getClosestVertexId(glm::vec3(goalPos, 0));

        pGameMapGraph->AStarSearch(pGameMapGraph->graphCDT, closestVertexIDToStart, closestVertexIDToGoal);
        Navigation.Path = pGameMapGraph->path;
        Navigation.finalPos = goalPos;
    }
}

void bee::navigation_system::CalculateNewPathForAgent(const entt::entity& agentEntity, const glm::vec2& goalPos)
{
    auto* Navigation = bee::Engine.ECS().Registry.try_get<bee::NavigationComponent>(agentEntity);
    auto* Body = bee::Engine.ECS().Registry.try_get<physics::BodyComponent>(agentEntity);

    if (Navigation && Body)
    {
        int closestVertexIDToStart = pGameMapGraph->getClosestVertexId(glm::vec3(Body->possition, 0));
        int closestVertexIDToGoal = pGameMapGraph->getClosestVertexId(glm::vec3(goalPos, 0));

        pGameMapGraph->AStarSearch(pGameMapGraph->graphCDT, closestVertexIDToStart, closestVertexIDToGoal);
        Navigation->Path = pGameMapGraph->path;
        Navigation->finalPos = goalPos;
    }
    else
    {
        std::cerr << "Agent does not have: ";
        if (Navigation == nullptr) std::cerr << "NavigationComponent ";
        if (Body == nullptr) std::cerr << "BodyComponent ";
        std::cerr << std::endl;
    }
}
