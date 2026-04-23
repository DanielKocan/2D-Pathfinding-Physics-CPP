#pragma once
#include "clipper2/clipper.h"
#include "tools/warnings.hpp"

BEE_DISABLE_WARNING_PUSH
BEE_DISABLE_WARNING_SIZE_T_CONVERSION
#include "cdt/CDT.h"
BEE_DISABLE_WARNING_POP

#include "core/ecs.hpp"
#include "core/engine.hpp"
#include "rendering/mesh.hpp"
#include "core/resources.hpp"
#include "rendering/debug_render.hpp"

#include "map.h"
#include "graph.h"

using namespace Clipper2Lib;
using namespace bee;
using namespace geometry2d;

class NavigationMesh
{
public:
    NavigationMesh(const std::shared_ptr<map>& _map);

    void drawGraph();
    void drawGraphPath();
    static void drawPaths(const std::vector<PathD>& _paths);
    void drawCDT();

    inline std::shared_ptr<graph> getGraphPtr() { return mapGraph; }
    inline graph getGraph() { return *mapGraph; }

    std::vector<PathD> solution;

private:
    void calculateCDT();
    static CDT::Triangulation<double> doTriangulation(const geometry2d::PolygonList& polygonList);
    static CDT::Triangulation<double> doTriangulation(const geometry2d::Polygon& polygon);

    static std::vector<PathD> ConvertToClipperData(const PolygonList& _polygonList);  // malePath funciton?
    static std::vector<PathD> UnionPaths(const std::vector<PathD>& _paths);
    static std::vector<glm::vec3> ConvertVerticesCDT(const std::vector<CDT::V2d<double>>& cdtVertices, double zValue = 0.0);
    static std::vector<glm::vec3> CalculateNormals(const CDT::Triangulation<double>& CDT);
    static std::vector<uint16_t> ConvertIndiciesFromCDT(const std::vector<CDT::Triangle>& triangles);
    static void AddSideWalls(float height,
                             std::vector<glm::vec3>& pos,
                             std::vector<glm::vec3>& normal,
                             std::vector<uint16_t>& indicies,
                             const Polygon& poly);
    void CreateMeshForObstacles();

    std::shared_ptr<map> mapFromTextFile;
    std::shared_ptr<graph> mapGraph;

    CDT::Triangulation<double> cdt;

    // for test
    CDT::Triangulation<double> meshCDT;

    std::vector<std::vector<glm::vec3>> debugNormals;
    std::vector<std::vector<glm::vec3>> debugPos;
};
