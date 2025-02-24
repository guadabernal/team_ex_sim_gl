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
    Room(float relx, float rely, float relw, float relh,
        float floor_width, float floor_height)
        : x(relx* floor_width),
        y(rely* floor_height),
        w(relw* floor_width),
        h(relh* floor_height)
    {}

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

struct Hole {
    float centerX;
    float centerY;
    float radius;
    float depth;
};

// This function generates a list of holes.
// Parameters:
//   rows, cols - grid size
//   scale     - cell size in meters
//   mu        - mean hole radius (in meters)
//   sigma     - standard deviation for hole radius (in meters)
//   numHoles  - number of holes to generate (if 0, a random count is chosen)
//   depth     - the depth of the hole (in meters)
inline std::vector<Hole> generateHolesList(int rows, int cols,
                                           float scale,
                                           float mu,
                                           float sigma,
                                           int numHoles,
                                           float depth = 1.0f)
{
    std::vector<Hole> holes;
    // Use a fixed seed (or vary it) for reproducibility.
    unsigned seed = 3;
    std::mt19937 rng(seed);

    // If no hole count specified, pick a random count between 5 and 15.
    if (numHoles == 0) {
        std::uniform_int_distribution<int> holeCountDist(5, 15);
        numHoles = holeCountDist(rng);
    }

    // Normal distribution for hole radius (in meters).
    std::normal_distribution<float> radiusDist(mu, sigma);
    // Uniform distributions to choose a random cell in the grid.
    std::uniform_int_distribution<int> rowDist(0, rows - 1);
    std::uniform_int_distribution<int> colDist(0, cols - 1);

    for (int i = 0; i < numHoles; i++) {
        int centerRow = rowDist(rng);
        int centerCol = colDist(rng);
        // Convert grid indices to meters (assuming each cell covers 'scale' meters)
        float centerX = (centerCol + 0.5f) * scale;
        float centerY = (centerRow + 0.5f) * scale;
        // Sample hole radius in meters (ensure at least one cell)
        float radiusMeters = std::max(radiusDist(rng), scale);
        Hole h;
        h.centerX = centerX;
        h.centerY = centerY;
        h.radius  = radiusMeters;
        h.depth   = depth;
        holes.push_back(h);
    }
    return holes;
}