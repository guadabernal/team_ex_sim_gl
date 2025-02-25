#pragma once

struct Lidar {
    const float maxRange = 2.0f;
    float cellSize = 0.5f;
    float sensorLastUpdateTimes = 0.0f;
    float defaultUpdateInterval = 0.1f;
    bool enabled = false;
    Lidar(float cellSize = 0.5f, bool enabled = false)
        : cellSize(cellSize), enabled(enabled), defaultUpdateInterval(defaultUpdateInterval) {}

    float meassure(const std::vector<std::vector<int>>& occupancy, float x, float y, float theta, float currentTime) {
        if (currentTime - sensorLastUpdateTimes < defaultUpdateInterval)
            return -1;
        sensorLastUpdateTimes = currentTime;        
        float step = cellSize * 0.5f;
        float distance = 0.0f;
        while (distance < maxRange) {
            float rayX = x + distance * std::cos(theta);
            float rayY = y + distance * std::sin(theta);
            int col = static_cast<int>(rayX / cellSize);
            int row = static_cast<int>(rayY / cellSize);
            if (row < 0 || row >= occupancy.size() ||
                col < 0 || col >= occupancy[0].size()) {
                break; // outside grid bounds
            }
            // Only treat walls (occupancy == 0) as obstacles.
            if (occupancy[row][col] == 0) {
                return distance;
            }
            // Ground (1) and holes (2) are ignored by the sensor.
            distance += step;
        }
        return maxRange;
    }
};


struct HeatSensor
{
    float cellSize = 0.5f;
    bool enabled = false;
    float sensorLastUpdateTimes = 0.0f;
    float defaultUpdateInterval = 0.01f;

    HeatSensor(float cellSize = 0.5f, bool enabled = false)
    : cellSize(cellSize), enabled(enabled), defaultUpdateInterval(defaultUpdateInterval) {}

    float meassure(const std::vector<std::vector<float>>& trueHeat, float x, float y, float currentTime) {
        if (currentTime - sensorLastUpdateTimes < defaultUpdateInterval)
            return -1;
        sensorLastUpdateTimes = currentTime;                
        int col = static_cast<int>(x / cellSize);
        int row = static_cast<int>(y / cellSize);
        // If the robot is out of bounds, return the base temperature (e.g., 20°C).
        if (row < 0 || row >= trueHeat.size() || col < 0 || col >= trueHeat[0].size())
            return 20.0f;
        float temp = trueHeat[row][col];
        // Clamp the temperature to a realistic range.
        if (temp < -40.0f)
            temp = -40.0f;
        if (temp > 125.0f)
            temp = 125.0f;
        return temp;
    }
};