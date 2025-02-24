#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <queue>
#include <cmath>
#include <cassert>
#include <office_elements.hpp>


struct OfficeDefinition
{
    std::vector<Room> rooms;
    std::vector<Wall> walls;
};

// ------------------- Example Layouts -------------------
OfficeDefinition office_01(float floor_width, float floor_height, float doorWidth)
{
    OfficeDefinition layout;
    Room room0(0.00f, 0.00f, 5.0f, 5.0f);
    layout.rooms.push_back(room0);

    Room room1 = Room(0.00, 0.00, 0.33, 0.50, floor_width, floor_height);
    room1.addDoor(3, 0.33 / 2 * floor_width, doorWidth);
    room1.addDoorRelativeFixWidth(0, 0.5, doorWidth);
    Room room2 = Room(0.33, 0.00, 0.33, 0.50, floor_width, floor_height);
    room2.addDoorRelativeFixWidth(3, 0.5, 2 * doorWidth);
    Room room3 = Room(0.66, 0.00, 0.34, 0.50, floor_width, floor_height);
    room3.addDoorRelativeFixWidth(2, 0.5, doorWidth);
    Room room4 = Room(0.00, 0.50, 1.00, 0.50, floor_width, floor_height);

    layout.rooms.push_back(room0);
    layout.rooms.push_back(room1);
    /*layout.rooms.push_back(room2);
    layout.rooms.push_back(room3);
    layout.rooms.push_back(room4);*/
    return layout;
}

OfficeDefinition office_02(float floor_width, float floor_height, float doorWidth)
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


// This function updates the occupancy grid to mark the holes.
// It loops over each hole in the list and, for cells within the hole's circle,
// if the cell is ground (value == 1) it sets it to 2.
inline void updateOccupancyWithHoles(std::vector<std::vector<int>>&grid,
    const std::vector<Hole>&holes,
    float scale)
{
    int rows = grid.size();
    if (rows == 0) return;
    int cols = grid[0].size();

    for (const Hole& h : holes) {
        // Compute the bounding box in grid indices.
        int colStart = std::max(0, static_cast<int>((h.centerX - h.radius) / scale));
        int colEnd = std::min(cols, static_cast<int>(std::ceil((h.centerX + h.radius) / scale)));
        int rowStart = std::max(0, static_cast<int>((h.centerY - h.radius) / scale));
        int rowEnd = std::min(rows, static_cast<int>(std::ceil((h.centerY + h.radius) / scale)));

        for (int i = rowStart; i < rowEnd; i++) {
            for (int j = colStart; j < colEnd; j++) {
                // Compute the cell center coordinates in meters.
                float cellCenterX = (j + 0.5f) * scale;
                float cellCenterY = (i + 0.5f) * scale;
                // Compute distance from cell center to the hole center.
                float dx = cellCenterX - h.centerX;
                float dy = cellCenterY - h.centerY;
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist <= h.radius) {
                    // Only mark ground cells (assumed to be 1) as holes (set to 2).
                    if (grid[i][j] == 1)
                        grid[i][j] = 2;
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
    OfficeDefinition def = office_02(cols * scale, rows * scale, doorWidth);
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

