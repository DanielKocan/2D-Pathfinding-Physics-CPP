#pragma once
#include "core/ecs.hpp"
#include "core/engine.hpp"
#include "core/input.hpp"
#include "core/transform.hpp"
#include "rendering/debug_render.hpp"
#include "tools/inspector.hpp"

#include "glm/vec3.hpp"

#include "physics/physics_components.h"
#include "ai/agent_components.h"

namespace bee
{

struct KeyboardControl
{
    bee::Input::KeyboardKey upKey;
    bee::Input::KeyboardKey downKey;
    bee::Input::KeyboardKey leftKey;
    bee::Input::KeyboardKey rightKey;
    bee::Input::KeyboardKey spaceKey;

    // Default constructor with WASD keys
    KeyboardControl()
        : upKey(bee::Input::KeyboardKey::W),
          downKey(bee::Input::KeyboardKey::S),
          leftKey(bee::Input::KeyboardKey::A),
          rightKey(bee::Input::KeyboardKey::D),
          spaceKey(bee::Input::KeyboardKey::Space)
    {
    }
};

class player_input_system : public bee::System
{
public:
    player_input_system() {};
    void Update(float deltaTime) override;
    void Render() override;
    static glm::vec2 GetMousePosInTheWorld();
};

}  // namespace bee
