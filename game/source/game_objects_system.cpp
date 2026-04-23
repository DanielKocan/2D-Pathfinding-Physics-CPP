#include "../include/game_objects_system.h"
#include "core/resources.hpp"
#include "iostream"
#include "rendering/mesh.hpp"
#include "rendering/model.hpp"
#include "rendering/render.hpp"

using namespace game;

void game_objects_system::Update(float) { BulletsLogic(); }

void game_objects_system::BulletsLogic()
{
    auto bulletsView = bee::Engine.ECS().Registry.view<const physics::DiskColliderComponent, game::BulletComponent>();

    for (auto [entity, DiskCollider, Bullet] : bulletsView.each())
    {
        for (const auto &Collision : DiskCollider.collisionData)
        {
            // std::cout << "Bullet collided with:" << static_cast<unsigned int>(Collision.collidedId) << "\n";
            if (Collision.collidedId != physics::WALLSID)
            {
                bee::Engine.ECS().DeleteEntity(Collision.collidedId);
                bee::Engine.ECS().DeleteEntity(entity);
            }
        }
        if (!DiskCollider.collisionData.empty())  // Alternatively its possible to use physics_system::HasExecutedFrame()
        {
            Bullet.numOfBounces -= 1;
            if (Bullet.numOfBounces <= 0)
            {
                bee::Engine.ECS().DeleteEntity(entity);
            }
        }
    }
}

void game_objects_system::SpawnBullet(glm::vec2 pos, glm::vec2 direction)
{
    // Create an entity
    auto bulletEntity = bee::Engine.ECS().CreateEntity();
    auto &entity = bee::Engine.ECS().CreateComponent<bee::Transform>(bulletEntity);
    entity.Name = "bullet";  // To see the name in the editor
    auto &entityBody = bee::Engine.ECS().CreateComponent<physics::BodyComponent>(bulletEntity, pos);
    auto &entityDiskCollider = bee::Engine.ECS().CreateComponent<physics::DiskColliderComponent>(bulletEntity, 0.2f);
    auto &entityBullet = bee::Engine.ECS().CreateComponent<game::BulletComponent>(bulletEntity, direction);

    // Load model
    const std::string modelPath = "models/BoxAndCylinder.gltf";
    const auto &boxy_model{bee::Model(bee::FileIO::Directory::SharedAssets, modelPath)};
    auto boxy_mesh{bee::Engine.Resources().Load<bee::Mesh>(boxy_model, 1)};
    bee::Engine.ECS().CreateComponent<bee::MeshRenderer>(bulletEntity, boxy_mesh, boxy_model.GetMaterials()[0]);

    entity.SetScale(glm::vec3(entityDiskCollider.r));
    entityBody.velocity = entityBullet.direction * entityBullet.movingSpeed;
}
