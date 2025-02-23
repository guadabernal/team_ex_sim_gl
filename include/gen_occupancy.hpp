#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <queue>
#include <cmath>
#include <cassert>

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

struct OfficeDefinition
{
    std::vector<Room> rooms;
    std::vector<Wall> walls;
};

// ------------------- Example Layouts -------------------
OfficeDefinition office_01(float floor_width, float floor_height, float doorWidth)
{
    OfficeDefinition layout;
    Room room0(0.00f, 0.00f, 20.0f, 20.0f);
    layout.rooms.push_back(room0);

    // Define sub-rooms
    Room room1(0.00f, 0.00f, 6.0f, 3.0f);
    room1.addDoorRelativeFixWidth(3, 0.2f, 0.9f);
    layout.rooms.push_back(room1);

    Room room2(6.0f, 0.00f, 4.0f, 3.0f);
    room2.addDoorRelativeFixWidth(0, 0.7f, 0.9f);
    layout.rooms.push_back(room2);

    Room room3(10.0f, 0.00f, 4.5f, 7.0f);
    room3.addDoorRelativeFixWidth(3, 0.2f, 0.9f);
    layout.rooms.push_back(room3);

    Room room4(14.5f, 0.00f, 5.5f, 7.0f);
    room4.addDoorRelativeFixWidth(3, 0.2f, 0.9f);
    layout.rooms.push_back(room4);

    Room room5(0.00f, 3.0f, 2.0f, 3.0f);
    room5.addDoorRelativeFixWidth(0, 0.5f, 0.9f);
    layout.rooms.push_back(room5);

    Room room6(6.0f, 3.0f, 4.0f, 2.0f);
    room6.addDoorRelativeFixWidth(2, 0.5f, 0.9f);
    layout.rooms.push_back(room6);

    Room room7(6.0f, 5.0f, 4.0f, 2.0f);
    room7.addDoorRelativeFixWidth(2, 0.5f, 0.9f);
    layout.rooms.push_back(room7);

    Room room8(0.00f, 12.0f, 6.0f, 8.0f);
    room8.addDoorRelativeFixWidth(1, 0.8f, 0.9f);
    layout.rooms.push_back(room8);

    Room room9(6.0f, 10.0f, 5.0f, 5.0f);
    room9.addDoorRelativeFixWidth(1, 0.8f, 0.9f);
    layout.rooms.push_back(room9);

    Room room10(6.0f, 15.0f, 5.0f, 5.0f);
    room10.addDoorRelativeFixWidth(0, 0.8f, 0.9f);
    layout.rooms.push_back(room10);

    
    layout.walls.push_back(Wall(5.0f, 3.0f, 5.0f, 8.0f));
    layout.walls.push_back(Wall(5.0f, 8.0f, 6.5f, 8.0f));

    return layout;
}


inline void addHoles(std::vector<std::vector<int>>& grid, float scale, float mu, float sigma, int numHoles)
{
    int rows = grid.size();
    if (rows == 0) return;
    int cols = grid[0].size();

    // Seed random number generator with a fixed or variable seed
    unsigned seed = 3;
    std::mt19937 rng(seed);

    // If no hole count was specified, pick a random count
    if (numHoles == 0) {
        std::uniform_int_distribution<int> holeCountDist(5, 15);
        numHoles = holeCountDist(rng);
    }

    // Normal distribution for hole radius (in meters).
    std::normal_distribution<float> radiusDist(mu, sigma);

    for (int i = 0; i < numHoles; i++) {
        // Pick a random center
        std::uniform_int_distribution<int> rowDist(0, rows - 1);
        std::uniform_int_distribution<int> colDist(0, cols - 1);
        int centerRow = rowDist(rng);
        int centerCol = colDist(rng);

        // Sample hole radius in meters (ensure at least 1 cell)
        float radiusMeters = std::max(radiusDist(rng), scale);
        int radiusCells = static_cast<int>(std::ceil(radiusMeters / scale));

        // Mark cells within that circular radius as holes (value=2)
        for (int r = std::max(0, centerRow - radiusCells);
            r < std::min(rows, centerRow + radiusCells + 1); r++)
        {
            for (int c = std::max(0, centerCol - radiusCells);
                c < std::min(cols, centerCol + radiusCells + 1); c++)
            {
                int dr = r - centerRow;
                int dc = c - centerCol;
                if (dr * dr + dc * dc <= radiusCells * radiusCells) {
                    if (grid[r][c] == 1) // only overwrite ground
                        grid[r][c] = 2;
                }
            }
        }
    }
}

static void drawWallSegment( std::vector<std::vector<int>>& grid, float scale, float wallSize, float x1, float y1, float x2, float y2) {
    // Convert to grid cells
    int rows = grid.size();
    int cols = grid[0].size();

    int ix1 = static_cast<int>(std::round(x1 / scale));
    int iy1 = static_cast<int>(std::round(y1 / scale));
    int ix2 = static_cast<int>(std::round(x2 / scale));
    int iy2 = static_cast<int>(std::round(y2 / scale));

    // Thickness in cells
    int t = std::max(1, (int)std::ceil(wallSize / scale));
    int halfT = t / 2;

    if (ix1 == ix2) {
        // vertical line
        int xMin = ix1 - halfT;
        int xMax = xMin + t;
        if (xMin < 0) xMin = 0;
        if (xMax > cols) xMax = cols;

        int yMin = std::min(iy1, iy2);
        int yMax = std::max(iy1, iy2);
        if (yMin < 0) yMin = 0;
        if (yMax > rows - 1) yMax = rows - 1;

        for (int yy = yMin; yy <= yMax; yy++) {
            for (int xx = xMin; xx < xMax; xx++) {
                grid[yy][xx] = 0; // 0 for wall
            }
        }
    }
    else if (iy1 == iy2) {
        // horizontal line
        int yMin = iy1 - halfT;
        int yMax = yMin + t;
        if (yMin < 0) yMin = 0;
        if (yMax > rows) yMax = rows;

        int xMin = std::min(ix1, ix2);
        int xMax = std::max(ix1, ix2);
        if (xMin < 0) xMin = 0;
        if (xMax > cols - 1) xMax = cols - 1;

        for (int yy = yMin; yy < yMax; yy++) {
            for (int xx = xMin; xx <= xMax; xx++) {
                grid[yy][xx] = 0;
            }
        }
    }
    else {
        int xMin = std::min(ix1, ix2);
        int xMax = std::max(ix1, ix2);
        int yMin = std::min(iy1, iy2);
        int yMax = std::max(iy1, iy2);

        if (xMin < 0) xMin = 0;
        if (xMax > cols - 1) xMax = cols - 1;
        if (yMin < 0) yMin = 0;
        if (yMax > rows - 1) yMax = rows - 1;

        for (int yy = yMin; yy <= yMax; yy++) {
            for (int xx = xMin; xx <= xMax; xx++) {
                // Could do a thickness check here
                grid[yy][xx] = 0;
            }
        }
    }
}

inline void generateOfficeMap(std::vector<std::vector<int>>& grid, float scale, float wallSize, float doorWidth) {
    int rows = grid.size();
    if (rows == 0) return;
    int cols = grid[0].size();
    assert(cols > 0);

    // Convert thicknesses to grid cells
    int wallThickness = std::max(1, (int)std::ceil(wallSize / scale));
    int doorWidthCells = std::max(1, (int)std::ceil(doorWidth / scale));
    int halfWall = wallThickness / 2;

    // 1) Initialize grid as free space (value=1)
    for (int j = 0; j < rows; ++j) {
        for (int i = 0; i < cols; ++i) {
            grid[j][i] = 1;
        }
    }

    // 2) Get both rooms and explicit walls from the layout
    OfficeDefinition def = office_01(cols * scale, rows * scale, doorWidth);
    auto& rooms = def.rooms;
    auto& extraWalls = def.walls;

    // 3) Build the rectangular boundary walls for each room
    int idx = 0;
    for (const auto& room : rooms) {
        ++idx;
        int x0 = (int)std::floor(room.x / scale);
        int y0 = (int)std::floor(room.y / scale);
        int x1 = (int)std::ceil((room.x + room.w) / scale);
        int y1 = (int)std::ceil((room.y + room.h) / scale);

        // Lambda that marks a rectangle in the grid as wall (0)
        auto buildRectWall = [&](int cx0, int cy0, int cx1, int cy1)
            {
                if (cx0 < 0) cx0 = 0;
                if (cy0 < 0) cy0 = 0;
                if (cx1 > cols) cx1 = cols;
                if (cy1 > rows) cy1 = rows;

                for (int yy = cy0; yy < cy1; ++yy) {
                    for (int xx = cx0; xx < cx1; ++xx) {
                        grid[yy][xx] = 0;
                    }
                }
            };

        // Top wall
        int topY = std::max(0, y0 - halfWall);
        buildRectWall(x0, topY, x1, topY + wallThickness);

        // Bottom wall
        int botY = std::min(rows - wallThickness, y1 - halfWall);
        buildRectWall(x0, botY, x1, botY + wallThickness);

        // Left wall
        int leftX = std::max(0, x0 - halfWall);
        buildRectWall(leftX, y0, leftX + wallThickness, y1);

        // Right wall
        int rightX = std::min(cols - wallThickness, x1 - halfWall);
        buildRectWall(rightX, y0, rightX + wallThickness, y1);
    }

    // 4) Carve out doorways in these room-walls
    for (const auto& room : rooms)
    {
        int x0 = (int)std::round(room.x / scale);
        int y0 = (int)std::round(room.y / scale);
        int x1 = (int)std::round((room.x + room.w) / scale);
        int y1 = (int)std::round((room.y + room.h) / scale);

        for (const auto& door : room.doors) {
            int doorStart = (int)std::round(door.offset / scale);
            int doorCells = (int)std::round(door.width / scale);

            if (door.wall == 0 || door.wall == 2) {
                // Right or left edges
                int cx0 = (door.wall == 2) ? std::max(0, x0 - halfWall) : std::min(cols - wallThickness, x1 - halfWall);

                // door goes from doorStart..doorStart+doorCells along the y-direction
                for (int yy = doorStart; yy < doorStart + doorCells; ++yy) {
                    for (int xx = cx0; xx < cx0 + wallThickness; ++xx) {
                        if (yy >= 0 && yy < rows && xx >= 0 && xx < cols) {
                            grid[yy][xx] = 1; // Freed for door
                        }
                    }
                }
            }
            else {
                // Top or bottom edges
                int cy0 = (door.wall == 1)
                    ? std::max(0, y0 - halfWall)
                    : std::min(rows - wallThickness, y1 - halfWall);

                // door goes from doorStart..doorStart+doorCells along the x-direction
                for (int yy = cy0; yy < cy0 + wallThickness; ++yy) {
                    for (int xx = doorStart; xx < doorStart + doorCells; ++xx) {
                        if (yy >= 0 && yy < rows && xx >= 0 && xx < cols) {
                            grid[yy][xx] = 1;
                        }
                    }
                }
            }
        }
    }

    // 5) draw the free-standing walls
    for (const auto& wall : extraWalls)
    {
        drawWallSegment(grid, scale, wallSize, wall.x1, wall.y1, wall.x2, wall.y2);
    }
}

