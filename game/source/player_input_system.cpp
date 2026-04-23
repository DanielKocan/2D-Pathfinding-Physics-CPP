#include "../include/player_input_system.h"
#include "../include/game_objects_system.h"
#include "ai/navigation_system.h"
#include "physics/physics_system.h"

void bee::player_input_system::Update(float deltaTime)
{
    auto playerView =
        bee::Engine.ECS()
            .Registry.view<const KeyboardControl, const bee::Transform, physics::BodyComponent, const game::PlayerComponent>();

    for (auto [entity, KeyboardControl, Transform, Body, Player] : playerView.each())
    {
        Body.velocity = glm::vec2(0.0f);

        // Entity movement
        if (bee::Engine.Input().GetKeyboardKey(KeyboardControl.downKey)) Body.velocity.y -= Player.movementSpeed * deltaTime;
        if (bee::Engine.Input().GetKeyboardKey(KeyboardControl.upKey)) Body.velocity.y += Player.movementSpeed * deltaTime;
        if (bee::Engine.Input().GetKeyboardKey(KeyboardControl.rightKey)) Body.velocity.x += Player.movementSpeed * deltaTime;
        if (bee::Engine.Input().GetKeyboardKey(KeyboardControl.leftKey)) Body.velocity.x -= Player.movementSpeed * deltaTime;

        if (bee::Engine.Input().GetMouseButtonOnce(Input::MouseButton::Left))
        {
            // std::cout << "Mouse pressed! ";

            glm::vec2 mousePosWorld = GetMousePosInTheWorld();
            glm::vec2 dir = glm::normalize(mousePosWorld - Body.possition);

            game::game_objects_system::SpawnBullet(Body.possition + dir, dir);
        }

        // Compute a new path
        if (bee::Engine.Input().GetKeyboardKeyOnce(KeyboardControl.spaceKey))
        {
            bee::navigation_system::CalculateNewPathForAllAgents(Body.possition);
        }
    }
}

void bee::player_input_system::Render()
{
    auto controlableView =
        bee::Engine.ECS().Registry.view<const KeyboardControl, const Transform, const physics::BodyComponent>();

    for (auto [entity, KeyboardControl, Transform, Body] : controlableView.each())
    {
        bee::Engine.DebugRenderer().AddCircle(bee::DebugCategory::Gameplay,
                                              glm::vec3(Body.possition, 0),
                                              0.4f,
                                              glm::vec4(0, 1, 0, 1));
    }
}

glm::vec2 bee::player_input_system::GetMousePosInTheWorld()
{
    // Get the NDC coordinates from the mouse position
    glm::vec2 NDC = Inspector::GetNdcMousePositionInGameViewport();
    glm::vec4 clipSpacePos(NDC.x, NDC.y, -1.0f, 1.0f);  // Adjusting to clip space

    glm::vec3 intersection(0.0f);

    // Retrieve the active camera's data from ECS
    for (const auto &[e, camera, cameraTransform] : bee::Engine.ECS().Registry.view<const bee::Camera, bee::Transform>().each())
    {
        // Compute the inverse projection matrix
        glm::mat4 invProjection = glm::inverse(camera.Projection);
        glm::vec4 viewSpacePos = invProjection * clipSpacePos;
        viewSpacePos.z = -1.0f;  // Set forward direction
        viewSpacePos.w = 0.0f;   // Direction vector

        // Compute the inverse view matrix
        const glm::mat4 &view = glm::inverse(cameraTransform.World());
        glm::vec3 rayDirection = glm::normalize(glm::vec3(view * viewSpacePos));  // Normalize the direction
        glm::vec3 rayOrigin = cameraTransform.GetTranslation();                   // Camera position in world space

        // Define the Z coordinate for 2D plane (typically Z = 0)
        float planeZ = 0.0f;

        // Calculate intersection with the 2D plane
        float t = (planeZ - rayOrigin.z) / rayDirection.z;  // t is the parameter for the ray equation

        // Ensure the intersection is in front of the camera
        intersection = rayOrigin + t * rayDirection;  // Get the intersection point in world space

        // std::cout << intersection.x << " " << intersection.y << std::endl;
    }

    return glm::vec2(intersection.x, intersection.y);
}
