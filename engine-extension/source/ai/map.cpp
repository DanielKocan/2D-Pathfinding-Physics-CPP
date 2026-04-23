#include "ai/map.h"

map::map(std::string _filePath) : filePath(std::move(_filePath)) { readAndSaveDataFromMap(); }

void map::readAndSaveDataFromMap()
{
    std::cout << "File path:" << filePath << std::endl;
    std::ifstream inputFile(filePath.c_str());

    if (!inputFile.is_open())
    {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return;
    }

    std::string line;

    while (std::getline(inputFile, line))
    {
        std::istringstream iss(line);
        char type = 0;
        glm::vec2 temp;
        Polygon tempPolygon;
        iss >> type;  // Read the first character to identify the type of data

        switch (type)
        {
            case 'w':
                while (iss >> temp.x >> temp.y)
                {
                    tempPolygon.push_back(temp);
                    std::cout << "Bounding box : " << temp.x << " " << temp.y << " ";
                }
                mapData.bBoxPos.push_back(tempPolygon);
                break;
            case 'o':
                while (iss >> temp.x >> temp.y)
                {
                    tempPolygon.push_back(temp);
                    std::cout << temp.x << " " << temp.y << " ";
                }
                mapData.obstacles.push_back(tempPolygon);
                std::cout << std::endl;
                break;
            case 'p':
                iss >> mapData.playerPos.x >> mapData.playerPos.y;
                break;
            case 'a':
                iss >> temp.x >> temp.y;
                mapData.agentPos.push_back(temp);
                break;
            default:
                break;
        }
    }
}

void map::drawMap()
{
    // Draw bounding box
    for (size_t i = 0; i < mapData.bBoxPos.size(); i++)
    {
        Polygon temp = mapData.bBoxPos[i];
        glm::vec2 start = temp[0];
        glm::vec2 tempStart = temp[0];

        for (size_t j = 1; j < temp.size(); j++)
        {
            bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, temp[j], glm::vec4(1, 0, 0, 1));
            start = temp[j];
        }

        // draw last connection with first and last point
        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, tempStart, temp.back(), glm::vec4(1, 0, 0, 1));
    }

    // Draw obstacles
    for (size_t i = 0; i < mapData.obstacles.size(); i++)
    {
        Polygon temp = mapData.obstacles[i];
        glm::vec2 start = temp[0];
        glm::vec2 tempStart = temp[0];

        for (size_t j = 1; j < temp.size(); j++)
        {
            bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, start, temp[j], glm::vec4(0, 0, 1, 1));
            start = temp[j];
        }

        // draw last connection with first and last point
        bee::Engine.DebugRenderer().AddLine(bee::DebugCategory::Gameplay, tempStart, temp.back(), glm::vec4(0, 0, 1, 1));
    }
}
