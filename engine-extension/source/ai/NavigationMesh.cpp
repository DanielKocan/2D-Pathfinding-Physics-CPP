#include "ai/NavigationMesh.h"
/*
On construct compute:
- all walkable areas minus all obstacles (using clipper)
- a triangulation of that space (using CDT)
- the dual graph of the CDT ()
*/

NavigationMesh::NavigationMesh(const std::shared_ptr<map>& _map)
{
    mapFromTextFile = _map;

    std::vector<PathD> walkablePaths = ConvertToClipperData(mapFromTextFile->mapData.bBoxPos);
    std::vector<PathD> obstaclesPaths = ConvertToClipperData(mapFromTextFile->mapData.obstacles);
    walkablePaths = UnionPaths(walkablePaths);
    obstaclesPaths = UnionPaths(obstaclesPaths);

    solution = Clipper2Lib::Difference(walkablePaths, obstaclesPaths, FillRule::NonZero);

    calculateCDT();

    mapGraph = std::make_shared<graph>(this->cdt);

    CreateMeshForObstacles();
}

void NavigationMesh::drawGraph() { mapGraph->drawGraph(mapGraph->graphCDT); }

void NavigationMesh::drawGraphPath() { mapGraph->drawPath(); }

std::vector<PathD> NavigationMesh::ConvertToClipperData(const PolygonList& _polygonList)
{
    std::vector<PathD> ResultPaths;
    for (const Polygon& polygon : _polygonList)
    {
        PathD path;
        for (const glm::vec2& point : polygon)
        {
            path.push_back(PointD(point.x, point.y));
        }
        ResultPaths.push_back(path);
    }
    return ResultPaths;
}

std::vector<PathD> NavigationMesh::UnionPaths(const std::vector<PathD>& _paths)
{
    std::vector<PathD> result;

    // result = Clipper2Lib::Union(_paths, _paths, FillRule::NonZero);
    result = Clipper2Lib::Union(_paths, FillRule::NonZero);

    return result;
}

std::vector<glm::vec3> NavigationMesh::ConvertVerticesCDT(const std::vector<CDT::V2d<double>>& cdtVertices, double zValue)
{
    std::vector<glm::vec3> glmVertices;

    for (const auto& vertex : cdtVertices)
    {
        glm::vec3 glmPoint(static_cast<float>(vertex.x), static_cast<float>(vertex.y), static_cast<float>(zValue));
        glmVertices.push_back(glmPoint);
    }

    return glmVertices;
}

std::vector<glm::vec3> NavigationMesh::CalculateNormals(const CDT::Triangulation<double>& CDT)
{
    std::vector<glm::vec3> normals(CDT.vertices.size(), glm::vec3(0.0f, 0.0f, 0.0f));  // Initialize normals for each vertex

    // Loop through each triangle to compute the normal
    for (const auto& triangle : CDT.triangles)
    {
        // Get the indices of the vertices that form the triangle
        int v0Index = triangle.vertices[0];
        int v1Index = triangle.vertices[1];
        int v2Index = triangle.vertices[2];

        // Get the triangle vertices
        glm::vec3 v0 = glm::vec3(CDT.vertices[v0Index].x, CDT.vertices[v0Index].y, 0.0f);
        glm::vec3 v1 = glm::vec3(CDT.vertices[v1Index].x, CDT.vertices[v1Index].y, 0.0f);
        glm::vec3 v2 = glm::vec3(CDT.vertices[v2Index].x, CDT.vertices[v2Index].y, 0.0f);

        // Calculate edge vectors
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;

        // Compute the cross product of the two edge vectors to get the normal
        glm::vec3 normal = glm::normalize(glm::cross(edge1, edge2));

        // Add this normal to each vertex of the triangle
        normals[v0Index] = normal;
        normals[v1Index] = normal;
        normals[v2Index] = normal;
    }

    return normals;
}

std::vector<uint16_t> NavigationMesh::ConvertIndiciesFromCDT(const std::vector<CDT::Triangle>& triangles)
{
    std::vector<uint16_t> vertices;
    const int numOfVerteciesInTriangle = 3;

    for (const auto& triangle : triangles)
    {
        for (int i = 0; i < numOfVerteciesInTriangle; i++)
        {
            vertices.push_back(static_cast<uint16_t>(triangle.vertices[i]));
        }
    }

    return vertices;
}

void NavigationMesh::CreateMeshForObstacles()
{
    for (size_t i = 0; i < mapFromTextFile->mapData.obstacles.size(); i++)
    {
        auto obstacleEntity = Engine.ECS().CreateEntity();
        auto& obstacleTransform = Engine.ECS().CreateComponent<Transform>(obstacleEntity);
        obstacleTransform.Name = "Obstacle" + std::to_string(i);
        obstacleTransform.SetTranslation(glm::vec3(0.0f));

        // CDT::Triangulation<double> meshCDT = doTriangulation(mapFromTextFile->mapData.obstacles);
        meshCDT = doTriangulation(mapFromTextFile->mapData.obstacles[i]);

        std::vector<glm::vec3> pos = ConvertVerticesCDT(meshCDT.vertices);
        std::vector<glm::vec3> normal = CalculateNormals(meshCDT);
        std::vector<uint16_t> indicies = ConvertIndiciesFromCDT(meshCDT.triangles);
        AddSideWalls(-2.0f, pos, normal, indicies, mapFromTextFile->mapData.obstacles[i]);

        std::shared_ptr<Mesh> obstacleMesh = Engine.Resources().Create<Mesh>();
        obstacleMesh->SetAttribute(Mesh::Attribute::Position, pos);
        obstacleMesh->SetAttribute(Mesh::Attribute::Normal, normal);
        obstacleMesh->SetIndices(indicies);

        auto obstacleMaterial = std::make_shared<Material>();
        obstacleMaterial->BaseColorFactor = glm::vec4(0.4f, 0.41f, 0.61f, 1.0f);
        Engine.ECS().CreateComponent<MeshRenderer>(obstacleEntity, obstacleMesh, obstacleMaterial);
        // Engine.ECS().CreateComponent<MeshRenderer>(obstacleEntity);
        debugNormals.push_back(normal);
        debugPos.push_back(pos);
    }
}

void NavigationMesh::AddSideWalls(float height,
                                  std::vector<glm::vec3>& pos,
                                  std::vector<glm::vec3>& normal,
                                  std::vector<uint16_t>& indicies,
                                  const Polygon& poly)
{
    auto newIndex = static_cast<uint16_t>(pos.size());

    Polygon newPoly = poly;

    if (IsClockwise(newPoly))
    {
        std::reverse(newPoly.begin(), newPoly.end());
    }

    for (int i = 0; i < static_cast<int>(poly.size()); i++)
    {
        int next_index = (i + 1) % poly.size();

        glm::vec3 v1Pos = glm::vec3(newPoly[i], 0.0f);
        glm::vec3 v2Pos = glm::vec3(newPoly[next_index], 0.0f);
        glm::vec3 newV1Pos = glm::vec3(v1Pos.x, v1Pos.y, height);
        glm::vec3 newV2Pos = glm::vec3(v2Pos.x, v2Pos.y, height);

        uint16_t newV1Index = newIndex;
        uint16_t newV2Index = newIndex + 1;
        uint16_t newV3Index = newIndex + 2;
        uint16_t newV4Index = newIndex + 3;
        newIndex += 4;

        pos.push_back(v1Pos);
        pos.push_back(newV1Pos);
        pos.push_back(newV2Pos);
        pos.push_back(v2Pos);

        // FirstTrirngle
        indicies.push_back(newV1Index);
        indicies.push_back(newV2Index);
        indicies.push_back(newV3Index);
        // SecondTriangle
        indicies.push_back(newV1Index);
        indicies.push_back(newV3Index);
        indicies.push_back(newV4Index);

        // Calculate edge vectors
        // glm::vec3 edge1 = v2Pos - v1Pos;
        // glm::vec3 edge2 = newV1Pos - v1Pos;

        // Compute the cross product of the two edge vectors to get the normal
        glm::vec3 newNormal = glm::normalize(glm::vec3(GetPerpendicularVector(v1Pos - v2Pos), 0.0f));

        // Add this normal to each vertex of the triangle
        for (int j = 0; j < 4; j++) normal.push_back(newNormal);
    }
}

void NavigationMesh::drawPaths(const std::vector<PathD>& _paths)
{
    for (size_t i = 0; i < _paths.size(); i++)
    {
        const auto& path = _paths[i];
        glm::vec2 start = glm::vec2(path[0].x, path[0].y);
        glm::vec2 tempStart = glm::vec2(path[0].x, path[0].y);

        for (size_t j = 1; j < path.size(); j++)
        {
            glm::vec2 temp = glm::vec2(path[j].x, path[j].y);
            bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, temp, glm::vec4(1, 0, 0, 1));
            start = temp;
        }

        // draw last connection with first and last point
        glm::vec2 back = glm::vec2(path.back().x, path.back().y);
        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, tempStart, back, glm::vec4(1, 0, 0, 1));
    }
}

void NavigationMesh::drawCDT()
{
    /*
    // basic draw for test
    for (const auto& edge : cdt.fixedEdges)
    {
        glm::vec2 start = glm::vec2(cdt.vertices[edge.v1()].x, cdt.vertices[edge.v1()].y);
        glm::vec2 end = glm::vec2(cdt.vertices[edge.v2()].x, cdt.vertices[edge.v2()].y);

        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, end, glm::vec4(1, 1, 1, 1));
    }*/

    // Triangles draw
    for (const auto& triangle : cdt.triangles)
    {
        auto index = triangle.vertices[0];
        glm::vec2 start = glm::vec2(cdt.vertices[index].x, cdt.vertices[index].y);
        index = triangle.vertices[1];
        glm::vec2 end = glm::vec2(cdt.vertices[index].x, cdt.vertices[index].y);
        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, end, glm::vec4(1, 1, 1, 1));

        index = triangle.vertices[1];
        start = glm::vec2(cdt.vertices[index].x, cdt.vertices[index].y);
        index = triangle.vertices[2];
        end = glm::vec2(cdt.vertices[index].x, cdt.vertices[index].y);
        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, end, glm::vec4(1, 1, 1, 1));

        index = triangle.vertices[2];
        start = glm::vec2(cdt.vertices[index].x, cdt.vertices[index].y);
        index = triangle.vertices[0];
        end = glm::vec2(cdt.vertices[index].x, cdt.vertices[index].y);
        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, end, glm::vec4(1, 1, 1, 1));
    }

    // Triangles draw FOR TEST ONLY
    for (const auto& triangle : meshCDT.triangles)
    {
        auto index = triangle.vertices[0];
        glm::vec2 start = glm::vec2(meshCDT.vertices[index].x, meshCDT.vertices[index].y);
        index = triangle.vertices[1];
        glm::vec2 end = glm::vec2(meshCDT.vertices[index].x, meshCDT.vertices[index].y);
        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, end, glm::vec4(0, 1, 0, 1));

        index = triangle.vertices[1];
        start = glm::vec2(meshCDT.vertices[index].x, meshCDT.vertices[index].y);
        index = triangle.vertices[2];
        end = glm::vec2(meshCDT.vertices[index].x, meshCDT.vertices[index].y);
        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, end, glm::vec4(0, 1, 0, 1));

        index = triangle.vertices[2];
        start = glm::vec2(meshCDT.vertices[index].x, meshCDT.vertices[index].y);
        index = triangle.vertices[0];
        end = glm::vec2(meshCDT.vertices[index].x, meshCDT.vertices[index].y);
        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, end, glm::vec4(0, 1, 0, 1));
    }

    for (size_t i = 0; i < debugNormals.size(); i++)
    {
        for (size_t j = 0; j < debugNormals[i].size(); j++)
        {
            glm::vec2 start = glm::vec2(debugPos[i][j].x, debugPos[i][j].y);
            glm::vec2 end = glm::vec2(debugPos[i][j].x + debugNormals[i][j].x, debugPos[i][j].y + debugNormals[i][j].y);

            // bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, end, glm::vec4(1, 0, 0, 1));
        }
    }
}

void NavigationMesh::calculateCDT()
{
    std::vector<CDT::V2d<double>> cdtVertices;
    std::vector<CDT::Edge> cdtEdges;

    int firstVertexIndex = 0;
    int currentIndex = 0;

    for (const PathD& polygon :
         solution)  // Ensuring each polygon's vertices are connected in sequence and that each polygon is closed.
    {
        firstVertexIndex = currentIndex;  // Remember the starting index for this polygon

        for (size_t i = 0; i < polygon.size(); i++)
        {
            const auto& point = polygon[i];
            CDT::V2d<double> temp = CDT::V2d<double>::make(point.x, point.y);
            cdtVertices.push_back(temp);

            if (i < polygon.size() - 1)  // If not the last point in the polygon
            {
                cdtEdges.push_back(CDT::Edge(currentIndex, currentIndex + 1));
            }
            else  // If it's the last point, connect to the first vertex of this polygon
            {
                cdtEdges.push_back(CDT::Edge(currentIndex, firstVertexIndex));
            }

            currentIndex++;
        }
    }

    cdt.insertVertices(cdtVertices);
    cdt.insertEdges(cdtEdges);
    cdt.eraseOuterTrianglesAndHoles();
}

CDT::Triangulation<double> NavigationMesh::doTriangulation(const PolygonList& polygonList)
{
    std::vector<CDT::V2d<double>> cdtVertices;
    std::vector<CDT::Edge> cdtEdges;
    CDT::Triangulation<double> finalCDT;

    int firstVertexIndex = 0;
    int currentIndex = 0;

    for (const Polygon& polygon :
         polygonList)  // Ensuring each polygon's vertices are connected in sequence and that each polygon is closed.
    {
        firstVertexIndex = currentIndex;  // Remember the starting index for this polygon

        for (size_t i = 0; i < polygon.size(); i++)
        {
            const auto& point = polygon[i];
            CDT::V2d<double> temp = CDT::V2d<double>::make(point.x, point.y);
            cdtVertices.push_back(temp);

            if (i < polygon.size() - 1)  // If not the last point in the polygon
            {
                cdtEdges.push_back(CDT::Edge(currentIndex, currentIndex + 1));
            }
            else  // If it's the last point, connect to the first vertex of this polygon
            {
                cdtEdges.push_back(CDT::Edge(currentIndex, firstVertexIndex));
            }

            currentIndex++;
        }
    }

    CDT::RemoveDuplicatesAndRemapEdges(cdtVertices, cdtEdges);
    finalCDT.insertVertices(cdtVertices);
    finalCDT.insertEdges(cdtEdges);
    finalCDT.eraseOuterTriangles();

    return finalCDT;
}

CDT::Triangulation<double> NavigationMesh::doTriangulation(const Polygon& polygon)
{
    std::vector<CDT::V2d<double>> cdtVertices;
    std::vector<CDT::Edge> cdtEdges;
    CDT::Triangulation<double> finalCDT;

    int firstVertexIndex = 0;
    int currentIndex = 0;

    firstVertexIndex = currentIndex;  // Remember the starting index for this polygon

    for (size_t i = 0; i < polygon.size(); i++)
    {
        const auto& point = polygon[i];
        CDT::V2d<double> temp = CDT::V2d<double>::make(point.x, point.y);
        cdtVertices.push_back(temp);

        if (i < polygon.size() - 1)  // If not the last point in the polygon
        {
            cdtEdges.push_back(CDT::Edge(currentIndex, currentIndex + 1));
        }
        else  // If it's the last point, connect to the first vertex of this polygon
        {
            cdtEdges.push_back(CDT::Edge(currentIndex, firstVertexIndex));
        }

        currentIndex++;
    }

    finalCDT.insertVertices(cdtVertices);
    finalCDT.insertEdges(cdtEdges);
    finalCDT.eraseOuterTriangles();

    return finalCDT;
}
