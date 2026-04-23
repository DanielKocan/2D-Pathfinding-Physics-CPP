#include "ai/graph.h"

template <typename T, typename priority_t>
struct PriorityQueue
{
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

    inline bool empty() const { return elements.empty(); }

    inline void put(T item, priority_t priority) { elements.emplace(priority, item); }

    T get()
    {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

template <typename Location>
std::vector<Location> reconstruct_path(Location start, Location goal, std::unordered_map<Location, Location> came_from)
{
    std::vector<Location> path;
    Location current = goal;
    if (came_from.find(goal) == came_from.end())
    {
        return path;  // no path can be found
    }
    while (current != start)
    {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start);  // optional
    std::reverse(path.begin(), path.end());
    return path;
}

graph::graph(const CDT::Triangulation<double>& _cdt)
{
    convertCDTTriangulationToGraph(_cdt);
    AStarSearch(this->graphCDT, 0, 2);
}

void graph::addVertex(int id, float x, float y)
{
    vertices[id] = MyVertex(id, glm::vec3(x, y, 0.0f));  // Z is set to 0.0 for 2D representation
}

void graph::drawGraph(const SimpleGraph& graph)
{
    for (const auto& vertex : vertices)
    {
        bee::Engine.DebugRenderer().AddCircle(bee::DebugCategory::Gameplay, vertex.second.pos, 0.05f, glm::vec4(1, 1, 0, 1));
    }

    for (const auto& [key, edges] : graph.edges)  // 'key' is the key of the unordered_map, which is an int | 'value' is the
                                                  // value of the unordered_map, which is a std::vector<std::pair<int, float>>
    {
        glm::vec3 from = vertices[key].pos;

        for (size_t i = 0; i < edges.size(); i++)
        {
            auto first_pair = edges[i];
            int id = first_pair.v;
            glm::vec3 to = vertices[id].pos;
            bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, from, to, glm::vec4(0, 0, 1, 1));
        }
    }
}

void graph::printPath()
{
    std::cout << "Path: ";
    for (size_t i = 0; i < path.size(); ++i)
    {
        std::cout << path[i];
        if (i + 1 != path.size()) std::cout << " -> ";
    }
    std::cout << std::endl;
}

void graph::drawPath()
{
    if (path.empty()) return;
    bee::Engine.DebugRenderer().AddCircle(bee::DebugCategory::Gameplay, vertices.at(0).pos, 0.15f, glm::vec4(0, 1, 0, 1));
    bee::Engine.DebugRenderer().AddCircle(bee::DebugCategory::Gameplay,
                                          vertices.at(path.back()).pos,
                                          0.15f,
                                          glm::vec4(1, 0.984, 0, 1));

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        glm::vec2 start = vertices[path[i]].pos;
        glm::vec2 end = vertices[path[i + 1]].pos;
        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, end, glm::vec4(0, 1, 0, 1));
    }
}

void graph::convertCDTTriangulationToGraph(const CDT::Triangulation<double>& _cdt)
{
    const int MAXTRIANGLECORNERS = 3;
    const int MAXTRIANGLENEIGHBOURS = 3;
    PolygonList triangleList;

    for (size_t i = 0; i < _cdt.triangles.size(); i++)
    {
        Polygon triangle;
        for (int j = 0; j < MAXTRIANGLECORNERS; j++)
        {
            auto index = _cdt.triangles[i].vertices[j];
            triangle.push_back(glm::vec2(_cdt.vertices[index].x, _cdt.vertices[index].y));
        }

        std::vector<Edge> edge;

        for (int k = 0; k < MAXTRIANGLENEIGHBOURS; k++)
        {
            if (_cdt.triangles[i].neighbors[k] != std::numeric_limits<CDT::TriInd>::max())
            {
                Edge tempEdge;
                tempEdge.v = _cdt.triangles[i].neighbors[k];

                glm::vec2 distance1 = ComputeCenterOfPolygon(triangle);
                Polygon traingleForDistance;
                for (int j = 0; j < MAXTRIANGLECORNERS; j++)
                {
                    auto index = _cdt.triangles[tempEdge.v].vertices[j];
                    traingleForDistance.push_back(glm::vec2(_cdt.vertices[index].x, _cdt.vertices[index].y));
                }
                glm::vec2 distance2 = ComputeCenterOfPolygon(triangle);
                tempEdge.cost = glm::distance(distance1, distance2);

                edge.push_back(tempEdge);
            }
        }

        graphCDT.edges.insert({static_cast<int>(i), edge});
        glm::vec2 centerOfTriangle = ComputeCenterOfPolygon(triangle);
        this->addVertex(static_cast<int>(i), centerOfTriangle.x, centerOfTriangle.y);
    }
}

void graph::AStarSearch(SimpleGraph graph, const int& start, const int& goal)
{
    // clear before using
    this->came_from.clear();
    this->cost_so_far.clear();
    this->path.clear();
    // Use queue when you need FIFO ordering and efficient operations at both ends.
    // Use priority_queue when you need to efficiently manage and access elements based on their priority.
    // std::priority_queue<int, std::greater<>> frontier;
    PriorityQueue<int, float> frontier;
    frontier.put(start, 0);

    // Use unordered_set when you need a colletions of unique elements with fast lookups and no specific order.
    // std::unordered_map<int, int> came_from; // path A->B is stored as came_from[B] == A
    // came_from[start] = start; // Try to add 'start' to the set 'reached'. If 'start' is not already in the set, it will be
    // added. If 'start' is already in the set, it will not be added again.
    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty())
    {
        int current = frontier.get();  // Returns a reference to the first (oldest) element in the queue without removing it.
        // frontier.pop(); // This function removes (oldest) the element at the front of the queue.

        if (current == goal)
        {
            // std::cout << "Found it!";
            path = reconstruct_path(start, goal, came_from);
            break;  // END if we found our goal.
        }

        // std::cout << "Visiting vertex: " << current << '\n';

        for (auto& next : graph.neighbors(current))
        {
            // It checks if the node next.v is not in the reached set.
            float new_cost = cost_so_far[current] + graph.cost(current, next.v);
            /*
            //std::cout << " next vertex: " << next.v << ", cost: " << next.cost << ", Heuristic: " << getHeuristic(next.v,
            goal) << std::endl; if (came_from.find(next.v) == came_from.end()) { frontier.push(next.v); came_from[next.v] =
            current;
            }*/

            if (cost_so_far.find(next.v) == cost_so_far.end() || new_cost < cost_so_far[next.v])
            {
                cost_so_far[next.v] = new_cost;

                float priority = new_cost + getHeuristic(next.v, goal);  // A star
                frontier.put(next.v, priority);

                came_from[next.v] = current;
            }
        }
    }
}

int graph::getClosestVertexId(glm::vec3 pos)
{
    int closestVertex = -1;
    float closestVertexDistance = std::numeric_limits<float>::max();

    for (const auto& vertex : vertices)
    {
        int key = vertex.first;

        float distance = glm::distance2(pos, vertex.second.pos);

        if (distance < closestVertexDistance)
        {
            closestVertexDistance = distance;
            closestVertex = key;
        }
    }
    return closestVertex;
}

int graph::getClosestVertexIdFromPath(glm::vec3 pos, const std::vector<int>& _path)
{
    int closestVertex = -1;
    float closestVertexDistance = std::numeric_limits<float>::max();

    for (const auto& vertex : _path)
    {
        float distance = glm::distance2(pos, vertices.at(vertex).pos);

        if (distance < closestVertexDistance)
        {
            closestVertexDistance = distance;
            closestVertex = vertex;
        }
    }
    return closestVertex;
}
