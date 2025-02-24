#pragma once

#include <vector>
#include <random>
#include <cmath>
#include <algorithm>


struct Door
{
    // wall index convention: 0:right, 1:top, 2:left, 3:bottom
    int wall;
    float offset;
    float width;

    Door(int wall, float offset, float width)
        : wall(wall), offset(offset), width(width) {}
};

struct Room
{
    float x, y;   // top-left coordinates in meters
    float w, h;   // width and height in meters
    std::vector<Door> doors;

    Room(float x, float y, float w, float h)
        : x(x), y(y), w(w), h(h) {}

    // Alternative constructor using relative coords
    Room(float relx, float rely, float relw, float relh, float floor_width, float floor_height)
        : x(relx* floor_width), y(rely* floor_height), w(relw* floor_width), h(relh* floor_height) {}

    void addDoor(int wall, float center, float width)
    {
        Door door(wall, center - width / 2.0f, width);
        doors.push_back(door);
    }

    // center and width are relative (0..1) to that edge's length
    void addDoorRelative(int wall, float relcenter, float relWidth)
    {
        float center = (wall == 0 || wall == 2)
            ? (y + relcenter * h)
            : (x + relcenter * w);
        float width = (wall == 0 || wall == 2)
            ? (relWidth * h)
            : (relWidth * w);
        addDoor(wall, center, width);
    }

    // center is relative, but door width is a fixed meter measure
    void addDoorRelativeFixWidth(int wall, float relcenter, float fixWidth)
    {
        float center = (wall == 0 || wall == 2)
            ? (y + relcenter * h)
            : (x + relcenter * w);
        addDoor(wall, center, fixWidth);
    }
};

struct Wall
{
    float x1, y1; // start point
    float x2, y2; // end point

    Wall(float x1, float y1, float x2, float y2)
        : x1(x1), y1(y1), x2(x2), y2(y2)
    {}
};

enum class HoleShape {
    CIRCLE,
    RECTANGLE
};

struct Hole {
    HoleShape shape;
    
    // For circular holes:
    float centerX;
    float centerY;
    float radius;
    
    // For rectangular holes (defined by top–left point):
    float topLeftX;
    float topLeftY;
    float width;
    float height;
    float depth;    // depth of the hole (positive value)

    // Constructor for circ holes
    Hole(HoleShape shape, float centerX, float centerY, float radius, float depth)
        : shape(shape), centerX(centerX), centerY(centerY), radius(radius), depth(depth), topLeftX(0.0f), topLeftY(0.0f), width(0.0f), height(0.0f) {}

    // Constructor for rect holes
    Hole(HoleShape shape, float topLeftX, float topLeftY, float width, float height, float depth)
        : shape(shape), topLeftX(topLeftX), topLeftY(topLeftY), width(width), height(height), depth(depth), centerX(0.0f), centerY(0.0f), radius(0.0f) {}

    // Default constructor
    Hole() : shape(HoleShape::CIRCLE), centerX(0.0f), centerY(0.0f), radius(0.0f), topLeftX(0.0f), topLeftY(0.0f), width(0.0f), height(0.0f), depth(0.0f) {}
};


// This function generates a list of holes.
// Parameters:
//   rows, cols - grid size
//   scale     - cell size in meters
//   mu        - mean hole radius (in meters)
//   sigma     - standard deviation for hole radius (in meters)
//   numHoles  - number of holes to generate (if 0, a random count is chosen)
//   depth     - the depth of the hole (in meters)
inline std::vector<Hole> generateHolesList(int rows, int cols, float scale, float mu, float sigma, int numHoles, float depth = 1.0f, bool rectangular = false) {
    std::vector<Hole> holes;
    unsigned seed = 3;
    std::mt19937 rng(seed);

    if (numHoles == 0) {
        std::uniform_int_distribution<int> holeCountDist(5, 15);
        numHoles = holeCountDist(rng);
    }

    if (!rectangular) {
        // Generate circular holes as before.
        std::normal_distribution<float> radiusDist(mu, sigma);
        std::uniform_int_distribution<int> rowDist(0, rows - 1);
        std::uniform_int_distribution<int> colDist(0, cols - 1);
        for (int i = 0; i < numHoles; i++) {
            int centerRow = rowDist(rng);
            int centerCol = colDist(rng);
            float centerX = (centerCol + 0.5f) * scale;
            float centerY = (centerRow + 0.5f) * scale;
            float radiusMeters = std::max(radiusDist(rng), scale);
            Hole h;
            h.shape = HoleShape::CIRCLE;
            h.centerX = centerX;
            h.centerY = centerY;
            h.radius = radiusMeters;
            h.depth = depth;
            // (width and height can be left unused or set to zero)
            h.width = 0;
            h.height = 0;
            holes.push_back(h);
        }
    }
    else {
        // Generate rectangular holes.
        std::normal_distribution<float> sizeDist(mu, sigma);
        std::uniform_int_distribution<int> rowDist(0, rows - 1);
        std::uniform_int_distribution<int> colDist(0, cols - 1);
        for (int i = 0; i < numHoles; i++) {
            int centerRow = rowDist(rng);
            int centerCol = colDist(rng);
            float centerX = (centerCol + 0.5f) * scale;
            float centerY = (centerRow + 0.5f) * scale;
            // For rectangles, sample half-sizes (ensuring at least one cell in size)
            float halfWidth = std::max(sizeDist(rng), scale);
            float halfHeight = std::max(sizeDist(rng), scale);
            Hole h;
            h.shape = HoleShape::RECTANGLE;
            h.centerX = centerX;
            h.centerY = centerY;
            h.width = 2 * halfWidth;
            h.height = 2 * halfHeight;
            h.depth = depth;
            // radius field is not used for rectangular holes.
            h.radius = 0;
            holes.push_back(h);
        }
    }
    return holes;
}

inline std::vector<Hole> generateHolesList_custom1() {
    std::vector<Hole> holes;
    // Reserve space for 8 holes.
    holes.reserve(8);

    holes.push_back(Hole(HoleShape::RECTANGLE, 14.5f, 2.0f, 0.5f, 1.5f, 1.0f));
    holes.push_back(Hole(HoleShape::RECTANGLE, 14.0f, 3.2f, 0.5f, 1.2f, 1.0f));
    holes.push_back(Hole(HoleShape::RECTANGLE, 7.0f, 7.0f, 1.5f, 0.2f, 1.0f));
    holes.push_back(Hole(HoleShape::RECTANGLE, 8.0f, 12.0f, 1.0f, 1.0f, 1.0f));

    holes.push_back(Hole(HoleShape::CIRCLE, 3.0f, 16.0f, 1.0f, 1.0f));
    holes.push_back(Hole(HoleShape::CIRCLE, 5.6f, 7.5f, 0.6f, 1.0f));
    holes.push_back(Hole(HoleShape::CIRCLE, 4.5f, 3.5f, 0.5f, 1.0f));

    return holes;
}

