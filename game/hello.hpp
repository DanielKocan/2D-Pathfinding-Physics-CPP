#pragma once

#include "glm/glm.hpp"
#include "core/ecs.hpp"
#include "core/input.hpp"

#include "ai/graph.h"
#include "ai/NavigationMesh.h"
#include "ai/agent_components.h"
#include "ai/navigation_system.h"
#include "ai/map.h"

#include "include/player_input_system.h"
#include "include/game_objects_system.h"
#include "include/game_components.h"

class Hello : public bee::System
{
public:
    Hello();
    ~Hello() override = default;
    void Update(float) override {};

    void InitGameClasses();
    void InitGameECS();

    void SpawnAgentsWithLimit(const int& numOfAgents = -1);  // -1 = maximum from file

private:
    std::shared_ptr<map> gameMap;
};
