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

    std::mt19937* rng_ptr;
    
    RescueRobot(float x, float y, float theta, float spawnTime, float cellSize, bool lidarEnabled, bool heatSensorEnabled, std::mt19937* rng_ptr_)
    : lidar(cellSize, lidarEnabled), heatSensor(cellSize, heatSensorEnabled),
      x(x), y(y), theta(theta), spawnTime(spawnTime), cellSize(cellSize), rng_ptr(rng_ptr_)
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

    void move_bak(float dt, const std::vector<std::vector<int>>& occupancy, const std::vector<RescueRobot>& robots,
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


    // Helper to get height at world coordinates (x,y) from a height grid.
    float getHeightAt(const std::vector<std::vector<float>>& heightMap, float x, float y, float cellSize)
    {
        int rows = heightMap.size();
        int cols = heightMap[0].size();
        int col = static_cast<int>(x / cellSize);
        int row = static_cast<int>(y / cellSize);
        // Clamp indices to grid bounds.
        row = std::max(0, std::min(row, rows - 1));
        col = std::max(0, std::min(col, cols - 1));
        return heightMap[row][col];
    }

    void move(float dt,
        const std::vector<std::vector<int>>& occupancy,
        const std::vector<RescueRobot>& robots,
        const std::vector<std::vector<int>>& trueOccupancy,
        std::vector<std::vector<int>>& unknownOccupancy,
        std::vector<std::vector<int>>& foundBy,
        const std::vector<std::vector<float>>& trueHeat,
        std::vector<std::vector<float>>& knownHeat,
        const std::vector<std::vector<float>>& trueHeight,  // height grid (meters)
        float currentTime)
    {
        // Do not update if not yet spawned or already dead.
        if (currentTime < spawnTime || dead)
            return;

        float prevX = x, prevY = y;

        // Introduce a small random heading perturbation.
        if (rotating) {
            const float ROTATE_DELTA = 0.1f;
            theta += ROTATE_DELTA;
        }
        else {
            std::uniform_real_distribution<float> angleDist(-0.1f, 0.1f);
            theta += angleDist(*rng_ptr);
        }

        // Compute candidate new position at full speed.
        float candidateX = x + v * std::cos(theta) * dt;
        float candidateY = y + v * std::sin(theta) * dt;

        // Compute the distance traveled (and ensure it is at least one cell to avoid division issues).
        float d = std::sqrt((candidateX - prevX) * (candidateX - prevX) +
            (candidateY - prevY) * (candidateY - prevY));
        float effective_d = std::max(d, cellSize);

        // Get the height at the current and candidate positions.
        float currentH = getHeightAt(trueHeight, prevX, prevY, cellSize);
        float candidateH = getHeightAt(trueHeight, candidateX, candidateY, cellSize);
        float slopeRatio = (candidateH - currentH) / effective_d; // positive: uphill, negative: downhill

        // Parameters.
        const float steepThreshold = 0.25f;  // 25% slope threshold
        const float PI = 3.14159265f;
        auto normalizeAngle = [PI](float angle) -> float {
            while (angle > PI)  angle -= 2 * PI;
            while (angle < -PI) angle += 2 * PI;
            return angle;
            };

        // If the local slope is too steep (either uphill or downhill), adjust heading.
        if (std::fabs(slopeRatio) > steepThreshold) {
            float eps = cellSize;  // use one cell as a small offset
            float hRight = getHeightAt(trueHeight, prevX + eps, prevY, cellSize);
            float hLeft = getHeightAt(trueHeight, prevX - eps, prevY, cellSize);
            float hDown = getHeightAt(trueHeight, prevX, prevY + eps, cellSize);
            float hUp = getHeightAt(trueHeight, prevX, prevY - eps, cellSize);
            float gradX = (hRight - hLeft) / (2 * eps);
            float gradY = (hDown - hUp) / (2 * eps);
            float desiredTheta;
            // If too steep uphill, steer toward downhill; if too steep downhill, steer toward uphill.
            if (slopeRatio > steepThreshold)
                desiredTheta = std::atan2(-gradY, -gradX); // downhill direction
            else
                desiredTheta = std::atan2(gradY, gradX);     // uphill direction
            float angleDiff = normalizeAngle(desiredTheta - theta);
            const float rotationGain = 0.1f;  // adjust as needed
            theta += rotationGain * angleDiff;
            rotating = true;
            // Recompute candidate new position using updated heading (full speed remains unchanged).
            candidateX = x + v * std::cos(theta) * dt;
            candidateY = y + v * std::sin(theta) * dt;
        }
        else {
            rotating = false;
        }

        // Update the robot's position.
        x = candidateX;
        y = candidateY;

        // Standard collision check.
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
            timeDeath = currentTime;
            return;
        }

        // Update sensor measurements.
        measure(occupancy, unknownOccupancy, foundBy, trueHeat, knownHeat, currentTime);
    }


};

int RescueRobot::static_id = 0;