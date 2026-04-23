#include "hello.hpp"

#include "core/engine.hpp"
#include "core/transform.hpp"
#include "rendering/debug_render.hpp"
#include "rendering/render.hpp"
#include "tools/inspector.hpp"

#include "rendering/mesh.hpp"
#include "rendering/model.hpp"
#include "core/resources.hpp"

#include "physics/physics_components.h"
#include "physics/physics_system.h"

Hello::Hello()
{
    InitGameClasses();
    InitGameECS();
    SpawnAgentsWithLimit();
}

void Hello::InitGameClasses() { gameMap = std::make_shared<map>("maps/map1.txt"); }

void Hello::InitGameECS()
{
    // Set up camera
    bee::Engine.ECS().CreateSystem<bee::Renderer>();
    auto cameraEntity = bee::Engine.ECS().CreateEntity();
    auto& transform = bee::Engine.ECS().CreateComponent<bee::Transform>(cameraEntity);
    transform.Name = "Camera";
    bee::Engine.ECS().CreateComponent<bee::Camera>(cameraEntity).Projection =
        glm::perspective(glm::radians(60.0f), 1.77f, 0.2f, 500.0f);
    auto view = glm::lookAt(glm::vec3(0, 3, 28), glm::vec3(0, 3, 0), glm::vec3(0, 1, 0));
    transform.SetFromMatrix(glm::inverse(view));

    // Player
    auto playerEntity = bee::Engine.ECS().CreateEntity();
    auto& entity = bee::Engine.ECS().CreateComponent<bee::Transform>(playerEntity);
    entity.Name = "Player";
    entity.SetScale(glm::vec3(0.4f));
    bee::Engine.ECS().CreateComponent<bee::KeyboardControl>(playerEntity);
    bee::Engine.ECS().CreateComponent<game::PlayerComponent>(playerEntity);
    bee::Engine.ECS().CreateComponent<physics::DiskColliderComponent>(playerEntity, 0.4f);
    bee::Engine.ECS().CreateComponent<physics::BodyComponent>(playerEntity,
                                                              glm::vec2(0.0f),
                                                              physics::BodyType::DYNAMIC,
                                                              false,
                                                              5.0f);
    // Load model for player
    const std::string modelPath = "models/BoxAndCylinder.gltf";
    const auto& boxy_model{bee::Model(bee::FileIO::Directory::SharedAssets, modelPath)};
    auto boxy_mesh{bee::Engine.Resources().Load<bee::Mesh>(boxy_model, 1)};
    bee::Engine.ECS().CreateComponent<bee::MeshRenderer>(playerEntity, boxy_mesh, boxy_model.GetMaterials()[0]);

    // Light
    auto light = bee::Engine.ECS().CreateEntity();
    bee::Engine.ECS().CreateComponent<bee::Light>(light, glm::vec3(1, 1, 1), 1000.f, 250.f, bee::Light::Type::Directional);
    auto& l_transform =
        bee::Engine.ECS().CreateComponent<bee::Transform>(light, glm::vec3(0, 0, 0), glm::vec3(1.f), glm::quat());
    l_transform.Name = "Light";

    // Main Game Systems
    bee::Engine.ECS().CreateSystem<bee::player_input_system>();
    bee::Engine.ECS().CreateSystem<physics::physics_system>(true, gameMap->mapData.obstacles);
    bee::Engine.ECS().CreateSystem<bee::navigation_system>(gameMap);
    bee::Engine.ECS().CreateSystem<game::game_objects_system>();

    bee::Engine.DebugRenderer().SetCategoryFlags((unsigned int)bee::DebugCategory::All);
}

void Hello::SpawnAgentsWithLimit(const int& numOfAgents)
{
    auto numberOfAgents = static_cast<size_t>(numOfAgents);

    // Spawn all agents from a map
    if (numOfAgents == -1) numberOfAgents = gameMap->mapData.agentPos.size();

    // Cap maximum amount of agents
    else if (numberOfAgents > gameMap->mapData.agentPos.size())
        numberOfAgents = gameMap->mapData.agentPos.size();

    for (size_t i = 0; i < numberOfAgents; i++)
    {
        // Create an entity
        auto agentEntity = bee::Engine.ECS().CreateEntity();

        // Give components for an entity
        auto& entity = bee::Engine.ECS().CreateComponent<bee::Transform>(agentEntity);
        entity.SetScale(glm::vec3(0.4, 0.4, 0.4));
        entity.Name = "Agent" + std::to_string(i);  // To see the name in the editor
        bee::Engine.ECS().CreateComponent<bee::NavigationComponent>(agentEntity);
        bee::Engine.ECS().CreateComponent<physics::BodyComponent>(agentEntity, gameMap->mapData.agentPos[i]);
        bee::Engine.ECS().CreateComponent<physics::DiskColliderComponent>(agentEntity, 0.4f);

        // Load model
        const std::string modelPath = "models/BoxAndCylinder.gltf";
        const auto& boxy_model{bee::Model(bee::FileIO::Directory::SharedAssets, modelPath)};
        auto boxy_mesh{bee::Engine.Resources().Load<bee::Mesh>(boxy_model, 1)};
        bee::Engine.ECS().CreateComponent<bee::MeshRenderer>(agentEntity, boxy_mesh, boxy_model.GetMaterials()[0]);
    }
}
