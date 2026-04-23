#pragma once
#include "core/ecs.hpp"
#include "core/engine.hpp"
#include "core/geometry2d.hpp"
#include "rendering/debug_render.hpp"
#include "string"

#include "iostream"
#include "fstream"
#include "sstream"

using namespace bee;
using namespace geometry2d;

struct MapData
{
    PolygonList bBoxPos;
    PolygonList obstacles;
    glm::vec2 playerPos;
    std::vector<glm::vec2> agentPos;
};

class map
{
public:
    map(std::string _filePath = "maps/map1.txt");
    void readAndSaveDataFromMap();
    void drawMap();
    std::string filePath;
    MapData mapData;
};
