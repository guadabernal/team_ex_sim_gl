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

// Recursive function to partition a rectangular area into smaller rooms.
// Parameters:
//   x, y     : top-left coordinates of the current region.
//   w, h     : width and height of the current region.
//   depth    : current recursion depth (controls how many splits occur).
//   minSize  : minimum allowed room dimension.
//   rng      : random number generator reference.
std::vector<Room> generateRandomOffices(float x, float y, float w, float h, int depth, float minSize, std::mt19937& rng) {
    std::vector<Room> rooms;
    std::uniform_real_distribution<float> chanceDist(0.0f, 1.0f);

    // Base condition:
    // - If maximum depth is reached.
    // - If the region is too small to split further.
    // - Or by chance (here a 20% chance to stop splitting even if it could be split).
    if (depth <= 0 || w < 2 * minSize || h < 2 * minSize || chanceDist(rng) > 0.8f) {
        rooms.push_back(Room(x, y, w, h));
        return rooms;
    }

    // Decide whether to split vertically or horizontally.
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    bool splitVertically = true;
    if (w / h >= 1.25f)
        splitVertically = true;
    else if (h / w >= 1.25f)
        splitVertically = false;
    else
        splitVertically = (dist(rng) > 0.5f);

    if (splitVertically) {
        // Choose a vertical split position ensuring both sides are at least minSize wide.
        std::uniform_real_distribution<float> splitDist(x + minSize, x + w - minSize);
        float splitX = splitDist(rng);
        // Recurse for the left and right partitions.
        auto leftRooms = generateRandomOffices(x, y, splitX - x, h, depth - 1, minSize, rng);
        auto rightRooms = generateRandomOffices(splitX, y, (x + w - splitX), h, depth - 1, minSize, rng);
        rooms.insert(rooms.end(), leftRooms.begin(), leftRooms.end());
        rooms.insert(rooms.end(), rightRooms.begin(), rightRooms.end());
    }
    else {
        // Choose a horizontal split position ensuring both parts are at least minSize tall.
        std::uniform_real_distribution<float> splitDist(y + minSize, y + h - minSize);
        float splitY = splitDist(rng);
        // Recurse for the top and bottom partitions.
        auto topRooms = generateRandomOffices(x, y, w, splitY - y, depth - 1, minSize, rng);
        auto bottomRooms = generateRandomOffices(x, splitY, w, (y + h - splitY), depth - 1, minSize, rng);
        rooms.insert(rooms.end(), topRooms.begin(), topRooms.end());
        rooms.insert(rooms.end(), bottomRooms.begin(), bottomRooms.end());
    }
    return rooms;
}

// Wrapper function to generate a random office layout for a given floor size.
// Parameters:
//   floor_width, floor_height : overall dimensions of the floor (in meters or your preferred units).
//   maxDepth                  : maximum number of recursive splits (default 4).
//   minRoomSize               : minimum room width/height (default 3.0).
std::vector<Room> generateRandomOfficeLayout(float floor_width, float floor_height, int maxDepth = 6, float minRoomSize = 3.0f) {
    // Seed the random number generator with the current time.
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 rng(seed);
    return generateRandomOffices(0, 0, floor_width, floor_height, maxDepth, minRoomSize, rng);
}

// Function to add random holes to the occupancy grid.
// grid: 2D grid where 1 represents ground.
// scale: cell size in meters.
// mu: average hole radius (in meters).
// sigma: standard deviation for the hole radius (in meters).
inline void addHoles(std::vector<std::vector<int>>& grid,
    float scale,
    float mu, float sigma)
{
    int rows = grid.size();
    if (rows == 0) return;
    int cols = grid[0].size();

    // Seed random number generator with current time.
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 rng(seed);

    // Randomly decide the number of holes (between 5 and 15).
    std::uniform_int_distribution<int> holeCountDist(5, 15);
    int numHoles = holeCountDist(rng);

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


// -----------------------------------------------------------------------------
// Generate a graph (as an adjacency list) where each room is a node and an edge
// is added between two rooms if they share a border (within some tolerance).
std::vector<std::vector<int>> generateRoomGraph(const std::vector<Room>& rooms, float tolerance = 0.01f) {
    std::vector<std::vector<int>> graph(rooms.size());
    // For each unique pair of rooms, check if they are adjacent.
    for (size_t i = 0; i < rooms.size(); ++i) {
        for (size_t j = i + 1; j < rooms.size(); ++j) {
            bool adjacent = false;

            // Get the edges of room i.
            float ri_left = rooms[i].x;
            float ri_right = rooms[i].x + rooms[i].w;
            float ri_top = rooms[i].y;
            float ri_bottom = rooms[i].y + rooms[i].h;

            // Get the edges of room j.
            float rj_left = rooms[j].x;
            float rj_right = rooms[j].x + rooms[j].w;
            float rj_top = rooms[j].y;
            float rj_bottom = rooms[j].y + rooms[j].h;

            // Check vertical adjacency:
            // If the right edge of one room is (nearly) the same as the left edge of the other,
            // and their vertical spans overlap.
            if (std::fabs(ri_right - rj_left) < tolerance || std::fabs(rj_right - ri_left) < tolerance) {
                float verticalOverlap = std::min(ri_bottom, rj_bottom) - std::max(ri_top, rj_top);
                if (verticalOverlap > 0)
                    adjacent = true;
            }
            // Check horizontal adjacency:
            // If the bottom edge of one room is (nearly) the same as the top edge of the other,
            // and their horizontal spans overlap.
            if (std::fabs(ri_bottom - rj_top) < tolerance || std::fabs(rj_bottom - ri_top) < tolerance) {
                float horizontalOverlap = std::min(ri_right, rj_right) - std::max(ri_left, rj_left);
                if (horizontalOverlap > 0)
                    adjacent = true;
            }
            if (adjacent) {
                graph[i].push_back(j);
                graph[j].push_back(i);
            }
        }
    }
    return graph;
}

// -----------------------------------------------------------------------------
// A simple BFS traversal over the room graph that prints the room indices.
void bfsTraversal(const std::vector<std::vector<int>>& graph, int start) {
    std::vector<bool> visited(graph.size(), false);
    std::queue<int> q;
    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        int current = q.front();
        q.pop();
        std::cout << "Visited room " << current << "\n";
        for (int neighbor : graph[current]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                q.push(neighbor);
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
        std::cout << idx << " " << room.x << ", " << room.y << ", " << room.w << ", " << room.h << std::endl;
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



// Function to set up a sample occupancy grid.
inline void generateSampleMap(std::vector<std::vector<int>> &grid) {
    int rows = grid.size();
    if (rows == 0) return;
    int cols = grid[0].size();

    // 1. Fill the grid with ground (value 1).
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x)
            grid[y][x] = 1;

    // 2. Create walls (value 0) along the border.
    for (int x = 0; x < cols; ++x) {
        grid[0][x] = 0;
        grid[rows - 1][x] = 0;
    }
    for (int y = 0; y < rows; ++y) {
        grid[y][0] = 0;
        grid[y][cols - 1] = 0;
    }

    // 3. Add a couple of holes (value 2) inside.
    if (rows > 4 && cols > 4) {
        grid[2][2] = 2;
        grid[rows / 2][cols / 2] = 2;
    }
}