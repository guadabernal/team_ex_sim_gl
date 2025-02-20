#pragma once

constexpr float PI = 3.14159265f;

struct VineRobot {
    // The vine's path as a list of points in world coordinates.
    std::vector<std::pair<float, float>> points;
    // The current intended expansion direction (in radians).
    float theta;
    // Expansion rate in meters per second.
    float expansionRate;
    // The index into 'points' that is used as the pivot for rotations.
    int pivotIndex;
    // Last update time for the tip sensor.
    float tipSensorLastUpdate;

    bool retracting = false;
    float retractRemaining = 0.0f;


    VineRobot()
        : theta(0.0f), expansionRate(0.5f), pivotIndex(0), tipSensorLastUpdate(0.0f)
    {
        // Initialize at a chosen base point.
        points.push_back({ 1.0f, 1.0f });
    }

    // Return the current tip (the last point).
    std::pair<float, float> tip() const {
        return points.back();
    }

    // Returns true if the given point collides with a wall.
    // (Assumes occupancy cell value 0 means wall; outside grid counts as collision.)
    bool isColliding(const std::pair<float, float>& pt, const std::vector<std::vector<int>>& occupancy, float cellSize) {
        int col = static_cast<int>(pt.first / cellSize);
        int row = static_cast<int>(pt.second / cellSize);
        if (row < 0 || row >= occupancy.size() || col < 0 || col >= occupancy[0].size())
            return true;
        return (occupancy[row][col] == 0);
    }

    // Rotate all points from pivotIndex onward about the pivot by deltaTheta.
    void rotateFromPivot(float deltaTheta) {
        auto pivot = points[pivotIndex];
        for (int i = pivotIndex; i < points.size(); i++) {
            float rx = points[i].first - pivot.first;
            float ry = points[i].second - pivot.second;
            float rrx = rx * std::cos(deltaTheta) - ry * std::sin(deltaTheta);
            float rry = rx * std::sin(deltaTheta) + ry * std::cos(deltaTheta);
            points[i].first = pivot.first + rrx;
            points[i].second = pivot.second + rry;
        }
        // Update the overall expansion direction.
        theta += deltaTheta;
    }

    // Check for collisions along the vine (excluding the tip).
    // Returns the index of the first colliding point (if any), or -1 if none.
    int firstCollidingIndex(const std::vector<std::vector<int>>& occupancy, float cellSize) {
        for (int i = pivotIndex + 1; i < points.size() - 1; i++) {
            if (isColliding(points[i], occupancy, cellSize))
                return i;
        }
        return -1;
    }

    // The move function (unchanged from our turning/expansion logic).
    void move(const std::vector<std::vector<int>>& occupancy, float cellSize, float dt) {
        // If currently retracting, retract gradually.
        if (retracting) {
            // Compute the retraction distance for this time step.
            float d_retract = expansionRate * dt;
            if (d_retract > retractRemaining)
                d_retract = retractRemaining;
            float remaining = d_retract;
            // Remove points from the tip until we've retracted 'd_retract' meters.
            while (remaining > 0 && points.size() > 1) {
                float dx = points.back().first - points[points.size() - 2].first;
                float dy = points.back().second - points[points.size() - 2].second;
                float segLength = std::sqrt(dx * dx + dy * dy);
                if (segLength <= remaining) {
                    remaining -= segLength;
                    points.pop_back();
                }
                else {
                    float ratio = (segLength - remaining) / segLength;
                    points.back().first = points[points.size() - 2].first + dx * ratio;
                    points.back().second = points[points.size() - 2].second + dy * ratio;
                    remaining = 0;
                }
            }
            retractRemaining -= d_retract;
            if (retractRemaining <= 0) {
                // Retraction complete: update pivot and rotate 60�.
                retracting = false;
                pivotIndex = static_cast<int>(points.size()) - 1;
                float turnAngle = 60.0f * (3.14159265f / 180.0f);
                theta += turnAngle;
            }
            return;
        }

        // Check for collision in non-tip segments.
        int collidingIndex = firstCollidingIndex(occupancy, cellSize);
        if (collidingIndex != -1) {
            int pointsFromTip = static_cast<int>(points.size()) - collidingIndex;
            if (pointsFromTip <= 5) {
                // Begin gradual retraction: set target of 10 cm.
                retracting = true;
                retractRemaining = 1.0f;
                return;
            }
            else {
                // Collision farther back�pause expansion.
                expansionRate = 0;
                return;
            }
        }

        // Normal tip extension.
        float d = expansionRate * dt;
        auto currentTip = tip();
        std::pair<float, float> candidate = {
            currentTip.first + d * std::cos(theta),
            currentTip.second + d * std::sin(theta)
        };

        if (!isColliding(candidate, occupancy, cellSize)) {
            points.push_back(candidate);
        }
        else {
            // If the tip is colliding, attempt a small sliding rotation.
            auto pivot = points[pivotIndex];
            float dx = currentTip.first - pivot.first;
            float dy = currentTip.second - pivot.second;
            float r = std::sqrt(dx * dx + dy * dy);
            if (r < 1e-4f)
                return;
            float maxAllowedRotation = d / r;
            float desiredRotation = 20.0f * (3.14159265f / 180.0f);
            float deltaTheta = std::min(desiredRotation, maxAllowedRotation);
            rotateFromPivot(deltaTheta);
            currentTip = tip();
            candidate = {
                currentTip.first + d * std::cos(theta),
                currentTip.second + d * std::sin(theta)
            };
            if (!isColliding(candidate, occupancy, cellSize))
                points.push_back(candidate);
        }
    }


    // --- New Sensor Functions for Lidar Measurement from the Tip ---

    // This sensor_lidar function mimics the RescueRobot's version
    // but uses the tip as the sensor location.
    // In simul.hpp inside the VineRobot struct, replace the sensor_lidar function with:
    std::vector<float> sensor_lidar(const std::vector<std::vector<int>>& occupancy, float cellSize) {
        const float max_range = 2.0f;
        float step = cellSize * 0.5f;
        std::vector<float> measurements(3, max_range);
        auto sensorPos = tip();
        float angleOffsets[3] = { -15.0f * (PI / 180.0f), 0.0f, 15.0f * (PI / 180.0f) };

        for (int i = 0; i < 3; i++) {
            float sensorAngle = theta + angleOffsets[i];
            float distance = 0.0f;
            while (distance < max_range) {
                float rayX = sensorPos.first + distance * std::cos(sensorAngle);
                float rayY = sensorPos.second + distance * std::sin(sensorAngle);
                int col = static_cast<int>(rayX / cellSize);
                int row = static_cast<int>(rayY / cellSize);
                if (row < 0 || row >= occupancy.size() ||
                    col < 0 || col >= occupancy[0].size()) {
                    break; // outside grid bounds
                }
                // Only treat walls (occupancy == 0) as obstacles.
                if (occupancy[row][col] == 0) {
                    measurements[i] = distance;
                    break;
                }
                distance += step;
            }
            // If no obstacle was encountered, set measurement to max_range.
            if (distance >= max_range) {
                measurements[i] = max_range;
            }
        }
        return measurements;
    }


    void measure(const std::vector<std::vector<int>>& trueOccupancy,
        std::vector<std::vector<int>>& unknownOccupancy,
        std::vector<std::vector<int>>& foundBy,
        const std::vector<std::vector<float>>& trueHeat,
        std::vector<std::vector<float>>& knownHeat,
        float cellSize, float currentTime)
    {
        const float defaultUpdateInterval = 0.1f;
        if (currentTime - tipSensorLastUpdate < defaultUpdateInterval)
            return;

        // Get three lidar measurements from the tip.
        std::vector<float> measurements = sensor_lidar(trueOccupancy, cellSize);
        if (measurements.empty())
            return;

        auto sensorPos = tip();
        const float maxRange = 2.0f;
        // Define the three sensor angle offsets (in radians) relative to the vine robot's heading.
        float angleOffsets[3] = { -15.0f * (PI / 180.0f), 0.0f, 15.0f * (PI / 180.0f) };

        // Loop over each sensor measurement.
        for (int i = 0; i < 3; i++) {
            float sensorAngle = theta + angleOffsets[i];
            float distance = measurements[i];
            int stepsCount = static_cast<int>(distance / cellSize);

            // Update free cells along the ray for this sensor.
            for (int j = 0; j < stepsCount; j++) {
                float rayX = sensorPos.first + (j * cellSize) * std::cos(sensorAngle);
                float rayY = sensorPos.second + (j * cellSize) * std::sin(sensorAngle);
                int col = static_cast<int>(rayX / cellSize);
                int row = static_cast<int>(rayY / cellSize);
                if (row >= 0 && row < unknownOccupancy.size() &&
                    col >= 0 && col < unknownOccupancy[0].size())
                {
                    // Only update if the cell is still undiscovered.
                    if (unknownOccupancy[row][col] == -1) {
                        unknownOccupancy[row][col] = 1; // ground
                        foundBy[row][col] = -2;         // mark as discovered by vine robot
                    }
                }
            }
            // If an obstacle was detected by this sensor, mark that cell as a wall.
            if (distance < maxRange) {
                float sensorX = sensorPos.first + distance * std::cos(sensorAngle);
                float sensorY = sensorPos.second + distance * std::sin(sensorAngle);
                int col = static_cast<int>(sensorX / cellSize);
                int row = static_cast<int>(sensorY / cellSize);
                if (row >= 0 && row < unknownOccupancy.size() &&
                    col >= 0 && col < unknownOccupancy[0].size())
                {
                    if (unknownOccupancy[row][col] == -1) {
                        unknownOccupancy[row][col] = 0; // wall
                        foundBy[row][col] = -2;         // mark as discovered by vine robot
                    }
                }
            }
        }
        tipSensorLastUpdate = currentTime;
    }
};
