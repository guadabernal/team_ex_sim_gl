#pragma once
#include <sensors.hpp>

struct RescueRobot {
    float x, y, theta;
    float size = 0.12f;
    float v = 0.2f;
    float battery = 100.0f;
    float timeDeath = 0.0f;
    float spawnTime = 0.0f;
    float cellSize = 0.5f;

    bool spawned = false;      // Has this robot been spawned?

    Lidar lidar;
    HeatSensor heatSensor;

    bool dead = false;         // Indicates if the robot fell in a hole
    bool rotating = false;     // Indicates if the robot is trying to rotate in place

    static int static_id; // dont work mutlithreaded
    int id = 0;
    
    RescueRobot(float x, float y, float theta, float spawnTime, float cellSize, bool lidarEnabled, bool heatSensorEnabled)
    : lidar(cellSize, lidarEnabled),
      heatSensor(cellSize, heatSensorEnabled),
      x(x), y(y), theta(theta), spawnTime(spawnTime), cellSize(cellSize)
    {
        id = RescueRobot::static_id;
        RescueRobot::static_id++;        
    }
    
    void measure(const std::vector<std::vector<int>>& trueOccupancy, std::vector<std::vector<int>>& unknownOccupancy, std::vector<std::vector<int>>& foundBy, 
        const std::vector<std::vector<float>>& trueHeat, std::vector<std::vector<float>>& knownHeat, float currentTime)
    {
        // ----- Distance Sensor Measurement (unchanged) -----
        if (lidar.enabled) {
            float distance = lidar.meassure(trueOccupancy, x, y, theta, currentTime);
            if (distance >= 0) {        
                int steps = static_cast<int>(distance / cellSize);
                for (int j = 0; j < steps; j++) {
                    float rayX = x + (j * cellSize) * std::cos(theta);
                    float rayY = y + (j * cellSize) * std::sin(theta);
                    int col = static_cast<int>(rayX / cellSize);
                    int row = static_cast<int>(rayY / cellSize);
                    if (row >= 0 && row < unknownOccupancy.size() &&
                        col >= 0 && col < unknownOccupancy[0].size()) {
                        if (unknownOccupancy[row][col] == -1) { // Only update if not discovered yet.
                            unknownOccupancy[row][col] = 1; // ground
                            foundBy[row][col] = id;
                        }
                    }
                }
                if (distance < lidar.maxRange) {
                    float sensorX = x + distance * std::cos(theta);
                    float sensorY = y + distance * std::sin(theta);
                    int col = static_cast<int>(sensorX / cellSize);
                    int row = static_cast<int>(sensorY / cellSize);
                    if (row >= 0 && row < unknownOccupancy.size() &&
                        col >= 0 && col < unknownOccupancy[0].size()) {
                        if (unknownOccupancy[row][col] == -1) {
                            unknownOccupancy[row][col] = 0; // wall
                            foundBy[row][col] = id;
                        }
                    }
                }
            }
        }

        // ----- Heat Sensor Measurement -----
        // Update only the cell where the robot is located with the direct temperature reading.
        if (heatSensor.enabled) {
            int col = static_cast<int>(x / cellSize);
            int row = static_cast<int>(y / cellSize);
            float heat = heatSensor.meassure(trueHeat, x, y, currentTime);
            if (heat > 0)
                knownHeat[row][col] = heat;
        }
    }


    int checkCollision(const std::vector<std::vector<int>>& occupancy, const std::vector<RescueRobot>& robots) {
        float halfSize = size / 2.0f;
        float left = x - halfSize;
        float right = x + halfSize;
        float top = y - halfSize;
        float bottom = y + halfSize;

        // Convert world coordinates to grid indices.
        int col_min = static_cast<int>(std::floor(left / cellSize));
        int col_max = static_cast<int>(std::floor(right / cellSize));
        int row_min = static_cast<int>(std::floor(top / cellSize));
        int row_max = static_cast<int>(std::floor(bottom / cellSize));

        int rows = occupancy.size();
        int cols = occupancy[0].size();
        if (col_min < 0) col_min = 0;
        if (row_min < 0) row_min = 0;
        if (col_max >= cols) col_max = cols - 1;
        if (row_max >= rows) row_max = rows - 1;

        bool wallFound = false;
        bool holeFound = false;
        // Check occupancy grid.
        for (int row = row_min; row <= row_max; ++row) {
            for (int col = col_min; col <= col_max; ++col) {
                int cell = occupancy[row][col];
                if (cell != 1) { // Not ground.
                    if (cell == 0)
                        wallFound = true;
                    else if (cell == 2)
                        holeFound = true;
                }
            }
        }
        if (wallFound) return 1;
        if (holeFound) return 2;

        // Check collision with other robots.
        for (const auto& other : robots) {
            // Skip self.
            if (&other == this || !other.spawned) continue;
            float otherHalf = other.size / 2.0f;
            float otherLeft = other.x - otherHalf;
            float otherRight = other.x + otherHalf;
            float otherTop = other.y - otherHalf;
            float otherBottom = other.y + otherHalf;
            // Check for overlap using axis-aligned bounding box (AABB) collision.
            bool overlapX = (x - halfSize < otherRight) && (x + halfSize > otherLeft);
            bool overlapY = (y - halfSize < otherBottom) && (y + halfSize > otherTop);
            if (overlapX && overlapY) return 1;
        }
        return 0;
    }

    void move(float dt, const std::vector<std::vector<int>>& occupancy, const std::vector<RescueRobot>& robots,
        const std::vector<std::vector<int>>& trueOccupancy, std::vector<std::vector<int>>& unknownOccupancy, std::vector<std::vector<int>>& foundBy,
        const std::vector<std::vector<float>>& trueHeat, std::vector<std::vector<float>>& knownHeat, float currentTime)
    {
        if (currentTime < spawnTime || dead) return;
        
        float prevX = x, prevY = y;

        if (rotating) {
            const float ROTATE_DELTA = 0.1f;
            theta += ROTATE_DELTA;
        }
        else {
            static std::default_random_engine rng(std::random_device{}());
            static std::uniform_real_distribution<float> angleDist(-0.1f, 0.1f);
            theta += angleDist(rng);
        }

        float candidateX = x + v * std::cos(theta) * dt;
        float candidateY = y + v * std::sin(theta) * dt;
        x = candidateX;
        y = candidateY;

        int collision = checkCollision(occupancy, robots);

        if (collision == 0) {
            rotating = false;
        }
        else if (collision == 1) {
            x = prevX;
            y = prevY;
            rotating = true;
            return;
        }
        else if (collision == 2) {
            v = 0;
            dead = true;
            timeDeath = currentTime;  // Record the actual death time.
            return;
        }

        // Update the discovered occupancy grid.
        measure(occupancy, unknownOccupancy, foundBy, trueHeat, knownHeat, currentTime);

    }   
};

int RescueRobot::static_id = 0;

//inline std::vector<RescueRobot> spawnRobots(Simulation& simulation, int nRobots, int sensorCount) {
//    std::vector<RescueRobot> robots;
//    robots.resize(nRobots);
//
//    int gridRows = simulation.known_grid.occupancy.size();
//    int gridCols = simulation.known_grid.occupancy[0].size();
//    float cellSize = simulation.known_grid.scale_m;
//
//    std::default_random_engine rng(std::random_device{}());
//    std::uniform_real_distribution<float> posXDist(0.0f, gridCols * cellSize);
//    std::uniform_real_distribution<float> posYDist(0.0f, gridRows * cellSize);
//    std::uniform_real_distribution<float> thetaDist(0.0f, 2 * PI);
//
//    float spawnPadding = 0.12f; // robot size as padding
//
//    for (int i = 0; i < robots.size(); i++) {
//        RescueRobot& robot = robots[i];
//        bool valid = false;
//        while (!valid) {
//            float candidateX = posXDist(rng);
//            float candidateY = posYDist(rng);
//
//            float left = candidateX - spawnPadding;
//            float right = candidateX + spawnPadding;
//            float top = candidateY - spawnPadding;
//            float bottom = candidateY + spawnPadding;
//
//            int col_min = static_cast<int>(std::floor(left / cellSize));
//            int col_max = static_cast<int>(std::floor(right / cellSize));
//            int row_min = static_cast<int>(std::floor(top / cellSize));
//            int row_max = static_cast<int>(std::floor(bottom / cellSize));
//
//            if (col_min < 0) col_min = 0;
//            if (row_min < 0) row_min = 0;
//            if (col_max >= gridCols) col_max = gridCols - 1;
//            if (row_max >= gridRows) row_max = gridRows - 1;
//
//            valid = true;
//            for (int row = row_min; row <= row_max && valid; ++row) {
//                for (int col = col_min; col <= col_max && valid; ++col) {
//                    if (simulation.known_grid.occupancy[row][col] != 1) {
//                        valid = false;
//                    }
//                }
//            }
//            if (valid) {
//                robot.x = candidateX;
//                robot.y = candidateY;
//            }
//        }
//        robot.theta = thetaDist(rng);
//        robot.v = 0.2f;
//        robot.battery = 100.0f;
//        robot.time_drop = 0;
//        robot.sensors.resize(sensorCount, 0);
//        robot.size = 0.12f;
//        robot.id = i;
//        robot.sensorLastUpdateTimes.resize(sensorCount, 0.0f);
//    }
//    return robots;
//}
