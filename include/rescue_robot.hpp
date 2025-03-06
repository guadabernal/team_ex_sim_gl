#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <sensors.hpp>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>

struct RescueRobot {
    // Pose and physical properties.
    float x, y, theta;
    float size = 0.12f;          // Also used as an approximate wheel base.
    float wheelDiameter = 0.06f; // For reference.
    float v = 0.2f;              // Maximum wheel speed (m/s).
    float battery = 100.0f;
    float timeDeath = 0.0f;
    float spawnTime = 0.0f;
    float cellSize = 0.5f;

    bool spawned = false;        // Has this robot been spawned?

    // Sensors.
    Lidar lidar;
    HeatSensor heatSensor;

    bool dead = false;           // Indicates if the robot fell in a hole

    static int static_id;        // (Not thread-safe.)
    int id = 0;

    std::mt19937* rng_ptr;

    // Rotation state for gradual turning.
    // rotationRemaining: angle (in radians) left to rotate.
    // A positive value means rotate clockwise,
    // negative means rotate counterclockwise.
    float rotationRemaining = 0.0f;

    RescueRobot(float x, float y, float theta, float spawnTime, float cellSize,
        bool lidarEnabled, bool heatSensorEnabled, std::mt19937* rng_ptr_)
        : lidar(cellSize, lidarEnabled), heatSensor(cellSize, heatSensorEnabled),
        x(x), y(y), theta(theta), spawnTime(spawnTime), cellSize(cellSize), rng_ptr(rng_ptr_)
    {
        id = RescueRobot::static_id;
        RescueRobot::static_id++;
    }

    // Sensor measurement (unchanged).
    void measure(const std::vector<std::vector<int>>& trueOccupancy,
        std::vector<std::vector<int>>& unknownOccupancy,
        std::vector<std::vector<int>>& foundBy,
        const std::vector<std::vector<float>>& trueHeat,
        std::vector<std::vector<float>>& knownHeat,
        float currentTime)
    {
        // ----- Lidar Measurement -----
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
                        if (unknownOccupancy[row][col] == -1) { // Only update if not discovered.
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
        if (heatSensor.enabled) {
            int col = static_cast<int>(x / cellSize);
            int row = static_cast<int>(y / cellSize);
            float heat = heatSensor.meassure(trueHeat, x, y, currentTime);
            if (heat > 0)
                knownHeat[row][col] = heat;
        }
    }

    // Collision checking (unchanged).
    // Returns:
    //   0 -> no collision,
    //   1 -> wall (or robot) collision,
    //   2 -> hole collision.
    int checkCollision(const std::vector<std::vector<int>>& occupancy,
        const std::vector<RescueRobot>& robots) {
        float halfSize = size / 2.0f;
        float left_bound = x - halfSize;
        float right_bound = x + halfSize;
        float top_bound = y - halfSize;
        float bottom_bound = y + halfSize;

        // Convert world coordinates to grid indices.
        int col_min = static_cast<int>(std::floor(left_bound / cellSize));
        int col_max = static_cast<int>(std::floor(right_bound / cellSize));
        int row_min = static_cast<int>(std::floor(top_bound / cellSize));
        int row_max = static_cast<int>(std::floor(bottom_bound / cellSize));

        int rows = occupancy.size();
        int cols = occupancy[0].size();
        if (col_min < 0) col_min = 0;
        if (row_min < 0) row_min = 0;
        if (col_max >= cols) col_max = cols - 1;
        if (row_max >= rows) row_max = rows - 1;

        bool wallFound = false;
        bool holeFound = false;
        // Check the occupancy grid.
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
            if (&other == this || !other.spawned) continue;
            float otherHalf = other.size / 2.0f;
            float otherLeft = other.x - otherHalf;
            float otherRight = other.x + otherHalf;
            float otherTop = other.y - otherHalf;
            float otherBottom = other.y + otherHalf;
            bool overlapX = (x - halfSize < otherRight) && (x + halfSize > otherLeft);
            bool overlapY = (y - halfSize < otherBottom) && (y + halfSize > otherTop);
            if (overlapX && overlapY)
                return 1;
        }
        return 0;
    }

    // ------------------------------------------------------------------
    // Arduino-like move() Function
    //
    // This function mimics an Arduino motor control function by taking two
    // normalized values (0 to 1) for the left and right wheels.
    // It calculates the new pose using differential drive kinematics,
    // checks for collisions, and updates sensor readings if the move is valid.
    //
    // Returns:
    //   0 -> move successful,
    //   1 -> wall (or robot) collision,
    //   2 -> hole encountered (robot is marked dead).
    // ------------------------------------------------------------------
    int move(float dt, float left, float right,
        const std::vector<std::vector<int>>& occupancy,
        const std::vector<RescueRobot>& robots,
        const std::vector<std::vector<int>>& trueOccupancy,
        std::vector<std::vector<int>>& unknownOccupancy,
        std::vector<std::vector<int>>& foundBy,
        const std::vector<std::vector<float>>& trueHeat,
        std::vector<std::vector<float>>& knownHeat,
        float currentTime)
    {
        // Do nothing if not yet spawned or if the robot is dead.
        if (currentTime < spawnTime || dead)
            return 0;

        // Save current pose.
        float prevX = x, prevY = y, prevTheta = theta;

        // Convert normalized motor commands to wheel speeds.
        float v_left = left * v;
        float v_right = right * v;

        // Differential drive kinematics.
        float linear = (v_left + v_right) / 2.0f;
        float angular = (v_right - v_left) / size;

        // Calculate the candidate new pose.
        float candidateX = x + linear * std::cos(theta) * dt;
        float candidateY = y + linear * std::sin(theta) * dt;
        float candidateTheta = theta + angular * dt;

        // Update the robot's pose.
        x = candidateX;
        y = candidateY;
        theta = candidateTheta;

        // Check for collisions.
        int collision = checkCollision(occupancy, robots);
        if (collision == 0) {
            // No collision: update sensors.
            measure(trueOccupancy, unknownOccupancy, foundBy, trueHeat, knownHeat, currentTime);
            return 0;
        }
        else if (collision == 1) {
            // Wall (or robot) collision: revert pose.
            x = prevX;
            y = prevY;
            theta = prevTheta;
            return 1;
        }
        else if (collision == 2) {
            // Hole encountered: revert pose, stop robot and mark as dead.
            x = prevX;
            y = prevY;
            theta = prevTheta;
            v = 0;
            dead = true;
            timeDeath = currentTime;
            return 2;
        }
        return 0;
    }

    // ------------------------------------------------------------------
    // Higher-Level Update() Function
    //
    // This function is meant to be called in your control loop.
    // It first checks whether the robot is currently performing a rotation.
    // If so, it rotates gradually. Otherwise, it commands forward motion.
    // When move() returns a collision, update() sets rotationRemaining to a random angle
    // between 90° and 180° (in radians), with the direction (left or right) chosen randomly.
    //
    // The rotation speed is computed using the physical model, but reduced to 1/8 of
    // the maximum in-place angular velocity:
    //   ω_max = 2*v / size
    // So the effective rotation speed is: (2*v / size) / 8.
    //
    // Additionally, when commanding forward motion, small random variations are added
    // to the left and right motor commands.
    // ------------------------------------------------------------------
    void update(float dt,
        const std::vector<std::vector<int>>& occupancy,
        const std::vector<RescueRobot>& robots,
        const std::vector<std::vector<int>>& trueOccupancy,
        std::vector<std::vector<int>>& unknownOccupancy,
        std::vector<std::vector<int>>& foundBy,
        const std::vector<std::vector<float>>& trueHeat,
        std::vector<std::vector<float>>& knownHeat,
        const std::vector<std::vector<float>>& trueHeight,
        float currentTime)
    {
        // If the robot is in a rotation phase, rotate gradually.
        if (rotationRemaining != 0.0f) {
            // Compute the reduced rotation speed:
            float physicalRotationSpeed = ((2.0f * v) / size) / 8.0f;  // rad/s
            // Determine the incremental rotation for this time step.
            float dTheta = std::min(std::abs(rotationRemaining), physicalRotationSpeed * dt);
            if (rotationRemaining > 0) {
                theta += dTheta;       // Rotate clockwise.
                rotationRemaining -= dTheta;
            }
            else {
                theta -= dTheta;       // Rotate counterclockwise.
                rotationRemaining += dTheta;
            }
            // Normalize theta to be within [-π, π].
            if (theta > M_PI)  theta -= 2 * M_PI;
            if (theta < -M_PI) theta += 2 * M_PI;
            // Optionally update sensors while rotating.
            measure(trueOccupancy, unknownOccupancy, foundBy, trueHeat, knownHeat, currentTime);
            return;
        }

        // Not currently rotating: command forward motion with random variations.
        std::uniform_real_distribution<float> speedVariationDist(-0.4f, 0.4f);
        float leftCommand = 1.0f + speedVariationDist(*rng_ptr);
        float rightCommand = 1.0f + speedVariationDist(*rng_ptr);
        // Clamp commands to [0, 1].
        leftCommand = std::clamp(leftCommand, 0.0f, 1.0f);
        rightCommand = std::clamp(rightCommand, 0.0f, 1.0f);

        int collisionCode = move(dt, leftCommand, rightCommand,
            occupancy, robots,
            trueOccupancy, unknownOccupancy, foundBy,
            trueHeat, knownHeat, currentTime);
        // If a collision occurs, set rotationRemaining to a random angle between 90° and 180°.
        if (collisionCode == 1) {
            std::uniform_real_distribution<float> angleDist(M_PI / 2.0f, M_PI);
            float randomAngle = angleDist(*rng_ptr);
            std::uniform_int_distribution<int> dirDist(0, 1);
            int randomDir = dirDist(*rng_ptr);
            if (randomDir == 0)
                rotationRemaining = randomAngle;   // Rotate right (clockwise).
            else
                rotationRemaining = -randomAngle;  // Rotate left (counterclockwise).
        }
    }
};

int RescueRobot::static_id = 0;





#if 0

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
            theta += ROTATE_DELTA * rotating;
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
            rotating = 0;
        }

        // Update the robot's position.
        x = candidateX;
        y = candidateY;

        // Standard collision check.
        int collision = checkCollision(occupancy, robots);
        if (collision == 0) {
            rotating = 0;
        }
        else if (collision == 1) {
            x = prevX;
            y = prevY;
            rotating = std::uniform_int_distribution<int>(0, 1)(*rng_ptr) * 2 - 1;;
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
#endif

