#pragma once
#include "core/engine.hpp"
#include "core/transform.hpp"
#include "rendering/debug_render.hpp"
#include "rendering/render.hpp"

#include "cdt/CDT.h"
#include "core/geometry2d.hpp"
#include "queue"
#include "unordered_set"
#include "iostream"
// #include "utility"// For std::pair

using namespace bee::geometry2d;

struct MyVertex
{
    int id;
    glm::vec3 pos;
    // vector of edges

    MyVertex(int id, glm::vec3 pos) : id(id), pos(pos) {}

    // Default constructor
    MyVertex() : id(-1), pos(glm::vec3(0.0f, 0.0f, 0.0f)) {}
};

struct Edge
{
    int v;
    float cost;  // cost of the edge
};

struct SimpleGraph
{
    // Efficient lookup but order is not needed
    std::unordered_map<int, std::vector<Edge>> edges;  // An edge from node firstInt(key) to node Edge.v with cost Edge.cost

    std::vector<Edge> neighbors(int id) { return edges[id]; }

    float cost(int v1, int v2)
    {
        // Find the vector of edges from vertex v1
        auto it = edges.find(v1);
        const std::vector<Edge>& edgeList = it->second;
        // Search for the edge with target vertex v2
        for (const auto& edge : edgeList)
        {
            if (edge.v == v2)
            {
                return edge.cost;
            }
        }
        return -1.0f;
    }
};

class graph
{
public:
    graph(const CDT::Triangulation<double>& _cdt);

    inline float heuristic(glm::vec2 a, glm::vec2 b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); }

    inline float getHeuristic(int v1, int v2) { return heuristic(vertices[v1].pos, vertices[v2].pos); }

    void addVertex(int id, float x, float y);
    // void addEdge(int v1, int v2, float cost);
    void drawGraph(const SimpleGraph& graph);
    void printPath();
    void drawPath();
    void convertCDTTriangulationToGraph(const CDT::Triangulation<double>& _cdt);

    void AStarSearch(SimpleGraph graph, const int& start, const int& goal);

    int getClosestVertexId(glm::vec3 pos);
    int getClosestVertexIdFromPath(glm::vec3 pos, const std::vector<int>& _path);

    std::unordered_map<int, MyVertex> vertices;

    std::unordered_map<int, int> came_from;
    std::unordered_map<int, float> cost_so_far;

    std::vector<int> path;
    SimpleGraph graphCDT;

private:
};
