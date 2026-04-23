#include <string>

#include "core/ecs.hpp"
#include "core/engine.hpp"
#include "hello.hpp"

using namespace bee;

int main(int, char**)
{
    Engine.Initialize();
    Engine.ECS().CreateSystem<Hello>();
    Engine.Run();
    Engine.Shutdown();
}
