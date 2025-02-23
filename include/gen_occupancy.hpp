#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <queue>


struct Door
{
    int wall;    // 0:right, 1:top, 2:left, 3:bottom
    float offset;   
    float width;
    Door(int wall, float offset, float width)
        : wall(wall), offset(offset), width(width) {}

};

struct Room
{
    float x, y;   // top-left coordinates in the grid
    float w, h;   // width and height
    std::vector<Door> doors;
    // x, y, w, h 
    Room(float x, float y, float w, float h)
        :x(x), y(y), w(w), h(h)
    {}
    // relx, rely, relw, relh, floor_width, floor_height
    Room(float relx, float rely, float relw, float relh, float floor_width, float floor_height)
        :x(relx * floor_width), y(rely * floor_height), w(relw * floor_width), h(relh * floor_height)
    {}

    void addDoor(int wall, float center, float width)
    {
        Door door(wall, center - width / 2, width);
        doors.push_back(door);
    }
    void addDoorRelative(int wall, float relcenter, float relWidth) {
        float center = wall == 0 || wall == 2 ? y + relcenter * h : x + relcenter * w;
        float width = wall == 0 || wall == 2 ? relWidth * h : relWidth * w;
        addDoor(wall, center, width);
    }
    void addDoorRelativeFixWidth(int wall, float relcenter, float fixWidth) {
        float center = wall == 0 || wall == 2 ? y + relcenter * h : x + relcenter * w;
        addDoor(wall, center, fixWidth);
    }

};


std::vector<Room> office_01(float floor_width, float floor_height, float doorWidth)
{
    Room room1 = Room(0.00, 0.00, 0.33, 0.50, floor_width, floor_height);
    room1.addDoor(3, 0.33/2 * floor_width, doorWidth);
    // room1.addDoorRelative(1, 0.5, doorWidth / room1.w);
    room1.addDoorRelativeFixWidth(0, 0.5, doorWidth);
    Room room2 = Room(0.33, 0.00, 0.33, 0.50, floor_width, floor_height);
    room2.addDoorRelativeFixWidth(3, 0.5, 2 * doorWidth);
    Room room3 = Room(0.66, 0.00, 0.34, 0.50, floor_width, floor_height);
    room3.addDoorRelativeFixWidth(2, 0.5, doorWidth);
    Room room4 = Room(0.00, 0.50, 1.00, 0.50, floor_width, floor_height);

    std::vector<Room> rooms;
    rooms.push_back(room1);
    rooms.push_back(room2);
    rooms.push_back(room3);
    rooms.push_back(room4);
    return rooms;
}

// Function to add random holes to the occupancy grid.
// grid: 2D grid where 1 represents ground.
// scale: cell size in meters.
// mu: average hole radius (in meters).
// sigma: standard deviation for the hole radius (in meters).
inline void addHoles(std::vector<std::vector<int>>& grid, float scale, float mu, float sigma, int numHoles)
{
    int rows = grid.size();
    if (rows == 0) return;
    int cols = grid[0].size();

    // Seed random number generator with current time.
    unsigned seed = 3; //  std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 rng(seed);

    if (numHoles == 0) {
        // Randomly decide the number of holes (between 5 and 15).
        std::uniform_int_distribution<int> holeCountDist(5, 15);
        numHoles = holeCountDist(rng);
    }

    // Normal distribution for hole radius (in meters).
    std::normal_distribution<float> radiusDist(mu, sigma);

    for (int i = 0; i < numHoles; i++) {
        // Randomly choose a center cell for the hole.
        std::uniform_int_distribution<int> rowDist(0, rows - 1);
        std::uniform_int_distribution<int> colDist(0, cols - 1);
        int centerRow = rowDist(rng);
        int centerCol = colDist(rng);

        // Sample the hole radius in meters and ensure it's at least one cell size.
        float radiusMeters = radiusDist(rng);
        radiusMeters = std::max(radiusMeters, scale);
        int radiusCells = static_cast<int>(std::ceil(radiusMeters / scale));

        // Loop over a bounding box around the center.
        for (int r = std::max(0, centerRow - radiusCells); r < std::min(rows, centerRow + radiusCells + 1); r++) {
            for (int c = std::max(0, centerCol - radiusCells); c < std::min(cols, centerCol + radiusCells + 1); c++) {
                int dr = r - centerRow;
                int dc = c - centerCol;
                // If within the circle (using Euclidean distance), mark as a hole.
                if (dr * dr + dc * dc <= radiusCells * radiusCells) {
                    // Only replace ground cells (value 1) with holes (value 2).
                    if (grid[r][c] == 1)
                        grid[r][c] = 2;
                }
            }
        }
    }
}


inline void generateOfficeMap(std::vector<std::vector<int>>& grid,
    float scale,         // cell size in meters
    float wallSize,      // desired wall thickness in meters
    float doorWidth)     // desired door opening width in meters
{
    // Get grid dimensions (assumed preallocated in cells).
    int rows = grid.size();
    if (rows == 0) return;
    int cols = grid[0].size();
    assert(cols > 0);

    // Convert parameters (meters to cells).
    int wallThickness = std::max(1, static_cast<int>(std::ceil(wallSize / scale)));
    int doorWidthCells = std::max(1, static_cast<int>(std::ceil(doorWidth / scale)));
    int halfWallThickness = wallThickness / 2;
    int d = wallThickness % 2 == 0 ? 0 : 1;
    // --- Step 1. Initialize grid as free space (1). ---
    for (int j = 0; j < rows; ++j)
        for (int i = 0; i < cols; ++i)
            grid[j][i] = 1;
   
    std::vector<Room> rooms = office_01(cols * scale, rows * scale, doorWidth);
    //std::vector<Room> rooms = generateRandomOfficeLayout(cols * scale, rows * scale);
    //std::vector<std::vector<int>> roomGraph = generateRoomGraph(rooms);
    //bfsTraversal(roomGraph, 0);
    
    // Walls
    int idx = 0;
    for (const auto &room : rooms) {
        //std::cout << idx << " " << room.x << ", " << room.y << ", " << room.w << ", " << room.h << std::endl;
        idx++;
        int x0 = room.x / scale;
        int y0 = room.y / scale;
        int x1 = (room.x + room.w) / scale;
        int y1 = (room.y + room.h) / scale;
        auto buildTheWall = [&](int cx0, int cy0, int cx1, int cy1) {
            for (int y = cy0; y < cy1; ++y)
                for (int x = cx0; x < cx1; ++x)
                    grid[y][x] = 0;
        };
        // top
        int cy0 = y0 > halfWallThickness ? y0 - halfWallThickness : 0;
        buildTheWall(x0, cy0, x1, cy0 + wallThickness);
        // bottom
        cy0 = y1 < rows - halfWallThickness ? y1 - halfWallThickness : rows - wallThickness;
        buildTheWall(x0, cy0, x1, cy0 + wallThickness);
        // left
        int cx0 = x0 > halfWallThickness ? x0 - halfWallThickness : 0;
        buildTheWall(cx0, y0, cx0 + wallThickness, y1);
        // rigth
        cx0 = x1 < cols - halfWallThickness ? x1 - halfWallThickness : cols - wallThickness;
        buildTheWall(cx0, y0, cx0 + wallThickness, y1);
    }
    // Doors
    for (const auto& room : rooms) {
        int x0 = static_cast<int>(std::round(room.x / scale));
        int y0 = static_cast<int>(std::round(room.y / scale));
        int x1 = static_cast<int>(std::round((room.x + room.w) / scale));
        int y1 = static_cast<int>(std::round((room.y + room.h) / scale));
        for (const auto& door : room.doors) {
            int doorStart = static_cast<int>(std::round(door.offset / scale));
            int doorEnd = doorStart + door.width / scale;
            if (door.wall == 0 || door.wall == 2) { // Door on the right or left walls.
                int cx0 = 0;
                if (door.wall == 2)
                   cx0 = cx0 = x0 > halfWallThickness ? x0 - halfWallThickness : 0;
                else
                   cx0 = x1 < cols - halfWallThickness ? x1 - halfWallThickness : cols - wallThickness;
                for (int y = doorStart; y < doorEnd; ++y)
                    for (int x = cx0; x < cx0 + wallThickness; ++x)
                        grid[y][x] = 1;
            }
            else { // Door on the top or bottom walls.
                int cy0 = 0;
                if (door.wall == 1)
                    cy0 = cy0 = y0 > halfWallThickness ? y0 - halfWallThickness : 0;
                else
                    cy0 = y1 < cols - halfWallThickness ? y1 - halfWallThickness : cols - wallThickness;
                for (int y = cy0; y < cy0 + wallThickness; ++y)
                    for (int x = doorStart; x < doorEnd; ++x)
                        grid[y][x] = 1;
            }
        }
    }
}