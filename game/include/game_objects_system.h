#pragma once
#include "core/ecs.hpp"
#include "core/engine.hpp"
#include "core/transform.hpp"
#include "rendering/debug_render.hpp"

#include "physics/physics_components.h"
#include "game_components.h"

namespace game
{
class game_objects_system : public bee::System
{
public:
    void Update(float deltaTime) override;
    static void BulletsLogic();

    static void SpawnBullet(glm::vec2 pos, glm::vec2 direction);
};
}  // namespace game
