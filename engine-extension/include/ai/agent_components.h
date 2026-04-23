#pragma once
#include "glm/vec2.hpp"
#include "string"
#include "vector"

namespace bee
{
struct NavigationComponent
{
    float lookAheadDistance = 2.5f;
    std::vector<int> Path;
    glm::vec2 attractionPoint = glm::vec2(0.0f, 0.0f);  // Empty vec2;
    glm::vec2 finalPos = glm::vec2(0.0f, 0.0f);
    float walkingSpeed = 2.5f;
};
}  // namespace bee
