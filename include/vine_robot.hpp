#ifndef VINE_ROBOT_HPP
#define VINE_ROBOT_HPP

#include <vector>
#include <cmath>
#include <utility>
#include <algorithm>
#include <iostream>
#include <random>

// -----------------------------------------
// Constants
// -----------------------------------------
constexpr float PI = 3.14159265f;

// For picking random turn angles
static constexpr float MAX_TURN_ANGLE_DEG = 4.0f;

// -----------------------------------------
// VineRobot (single-segment continuum)
// -----------------------------------------
struct VineRobot
{
    float baseX;
    float baseY;
    float baseTheta;
    float expansionRate;

    // Single-segment continuum params
    float length;
    float kappa;

    std::vector<std::pair<float, float>> points;
    bool reversing;
    float tipSensorLastUpdate;
    std::mt19937* rng_ptr;

    VineRobot()
        : baseX(0.0f), baseY(0.0f), baseTheta(0.0f), length(0.0f), kappa(0.0f), expansionRate(0.05f),
        reversing(false), tipSensorLastUpdate(0.0f), rng_ptr(nullptr), pendingAngleRad(0.0f)
    {
        points.clear();
        points.push_back({ baseX, baseY });
    }
    VineRobot(float startX, float startY, float initialAngleRad, std::mt19937* rng_ptr_)
        : baseX(startX), baseY(startY), baseTheta(initialAngleRad), length(0.0f),
        kappa(0.0f), expansionRate(0.05f), reversing(false), tipSensorLastUpdate(0.0f), rng_ptr(rng_ptr_)
    {
        points.clear();
        points.push_back({ baseX, baseY });
    }

    void move(const std::vector<std::vector<int>>& occupancy,
        float cellSize, float dt)
    {
        if (!reversing) {
            // -----------------------------
            // (A) Expanding forward
            // -----------------------------
            float oldLength = length;
            length += expansionRate * dt;

            // Recompute the shape with the new length
            recomputeArcPoints();

            // Check collision
            if (checkCollision(occupancy, cellSize)) {
                // Revert to old length => so we are no longer in collision
                length = oldLength;
                recomputeArcPoints();

                // Now begin reversing from here
                reversing = true;
                //std::cout << "[DEBUG] Collision => start reversing from length=" << length << "\n";
            }
            else {
                // Also check if the vine is forming a near loop back to its origin.
                // (Only perform this check if the vine is long enough.)
                if (length > 1.0f) {
                    auto tipPos = tip();
                    float dx = tipPos.first - baseX;
                    float dy = tipPos.second - baseY;
                    float distToBase = std::sqrt(dx * dx + dy * dy);
                    if (distToBase < 0.1f) {
                        // The tip is too close to the base (forming a near circle).
                        // Revert the expansion and initiate reversal.
                        length = oldLength;
                        recomputeArcPoints();
                        reversing = true;
                        std::cout << "[DEBUG] Vine tip too close to base (distance = " << distToBase
                            << " m). Initiating reversal to try a different angle.\n";
                    }
                }
            }
        }
        else {
            // -----------------------------
            // (B) Reversing
            // -----------------------------
            // Shrink a bit each frame
            float oldLength = length;
            length -= expansionRate * dt;
            if (length < 0.0f) {
                length = 0.0f;
            }

            recomputeArcPoints();

            // If fully retracted => pick a new random angle, expand again
            if (length <= 1e-6f) {
                length = 0.0f;
                reversing = false;

                std::uniform_real_distribution<float> angleDistDeg(-MAX_TURN_ANGLE_DEG, MAX_TURN_ANGLE_DEG);
                float angleDeg = angleDistDeg(*rng_ptr);
                float angleRad = angleDeg * (PI / 180.0f);
                std::uniform_real_distribution<float> coinDist(0.0f, 1.0f);
                float sign = (coinDist(*rng_ptr) < 0.5f) ? -1.0f : +1.0f;
                pendingAngleRad = angleRad * sign;
                kappa = 0.0f;

                //std::cout << "[DEBUG] Fully retracted => new random angle = " << (angleDeg * sign) << " deg => pendingAngleRad=" << pendingAngleRad << "\n";
            }
        }
        applyPendingAngleIfNeeded();
    }

    void recomputeArcPoints(int numSamples = 20)
    {
        points.clear();
        points.reserve(numSamples);

        if (numSamples < 1) numSamples = 1;
        float ds = (numSamples == 1 ? 0.0f : (length / (numSamples - 1)));

        for (int i = 0; i < numSamples; i++) {
            float s = ds * i;  // arc-length from base

            float xLocal, yLocal;
            if (std::fabs(kappa) < 1e-6f) {
                // near-zero => straight
                xLocal = s;
                yLocal = 0.0f;
            }
            else {
                xLocal = (1.0f / kappa) * std::sin(kappa * s);
                yLocal = (1.0f / kappa) * (1.0f - std::cos(kappa * s));
            }

            // Rotate by baseTheta
            float c = std::cos(baseTheta);
            float d = std::sin(baseTheta);
            float xWorld = baseX + (c * xLocal) - (d * yLocal);
            float yWorld = baseY + (d * xLocal) + (c * yLocal);

            points.push_back({ xWorld, yWorld });
        }
    }

    bool checkCollision(const std::vector<std::vector<int>>& occupancy, float cellSize) const
    {
        for (auto& p : points) {
            if (isColliding(p, occupancy, cellSize)) {
                return true;
            }
        }
        return false;
    }

    bool isColliding(const std::pair<float, float>& pt, const std::vector<std::vector<int>>& occupancy, float cellSize) const
    {
        int col = static_cast<int>(pt.first / cellSize);
        int row = static_cast<int>(pt.second / cellSize);

        // Out of bounds => collision
        if (row < 0 || row >= (int)occupancy.size() ||
            col < 0 || col >= (int)occupancy[0].size())
        {
            return true;
        }
        // occupancy=0 => wall => collision
        return (occupancy[row][col] == 0);
    }

    std::pair<float, float> tip() const
    {
        if (points.empty()) { return { baseX, baseY }; }
        return points.back();
    }

    float computeHeading() const
    {
        if (points.size() < 2) { return baseTheta; }
        auto& p1 = points[points.size() - 2];
        auto& p2 = points.back();
        float dx = p2.first - p1.first;
        float dy = p2.second - p1.second;
        return std::atan2(dy, dx);
    }

    std::vector<float> sensor_lidar(const std::vector<std::vector<int>>& occupancy, float cellSize)
    {
        const float max_range = 2.0f;
        float step = cellSize * 0.5f;
        std::vector<float> measurements(3, max_range);
        auto sensorPos = tip();

        float angleOffsets[3] = { -15.0f * (PI / 180.0f),
                                   0.0f,
                                   15.0f * (PI / 180.0f) };

        float heading = computeHeading();
        for (int i = 0; i < 3; i++) {
            float sensorAngle = heading + angleOffsets[i];
            float distance = 0.0f;
            while (distance < max_range) {
                float rayX = sensorPos.first + distance * std::cos(sensorAngle);
                float rayY = sensorPos.second + distance * std::sin(sensorAngle);

                int col = (int)(rayX / cellSize);
                int row = (int)(rayY / cellSize);

                if (row < 0 || row >= (int)occupancy.size() ||
                    col < 0 || col >= (int)occupancy[0].size())
                {
                    // out of bounds => stop
                    break;
                }
                if (occupancy[row][col] == 0) {
                    measurements[i] = distance;
                    break;
                }
                distance += step;
            }
            if (distance >= max_range) {
                measurements[i] = max_range;
            }
        }
        return measurements;
    }

    void measure(const std::vector<std::vector<int>>& trueOccupancy, std::vector<std::vector<int>>& unknownOccupancy, std::vector<std::vector<int>>& foundBy,
        const std::vector<std::vector<float>>&, std::vector<std::vector<float>>&, float cellSize, float currentTime)
    {
        const float defaultUpdateInterval = 0.1f;
        if (currentTime - tipSensorLastUpdate < defaultUpdateInterval) { return; }

        std::vector<float> measurements = sensor_lidar(trueOccupancy, cellSize);
        if (measurements.empty()) return;

        auto sensorPos = tip();
        float angleOffsets[3] = { -15.0f * (PI / 180.0f),
                                   0.0f,
                                   15.0f * (PI / 180.0f) };

        float heading = computeHeading();
        for (int i = 0; i < 3; i++) {
            float sensorAngle = heading + angleOffsets[i];
            float distance = measurements[i];
            int stepsCount = (int)(distance / cellSize);

            // Mark free cells along the ray
            for (int j = 0; j < stepsCount; j++) {
                float rayX = sensorPos.first + (j * cellSize) * std::cos(sensorAngle);
                float rayY = sensorPos.second + (j * cellSize) * std::sin(sensorAngle);
                int col = (int)(rayX / cellSize);
                int row = (int)(rayY / cellSize);

                if (row >= 0 && row < (int)unknownOccupancy.size() &&
                    col >= 0 && col < (int)unknownOccupancy[0].size())
                {
                    if (unknownOccupancy[row][col] == -1) {
                        unknownOccupancy[row][col] = 1; // ground
                        foundBy[row][col] = -2;         // discovered by vine
                    }
                }
            }

            // Mark wall if within maxRange
            const float maxRange = 2.0f;
            if (distance < maxRange) {
                float sensorX = sensorPos.first + distance * std::cos(sensorAngle);
                float sensorY = sensorPos.second + distance * std::sin(sensorAngle);
                int col = (int)(sensorX / cellSize);
                int row = (int)(sensorY / cellSize);

                if (row >= 0 && row < (int)unknownOccupancy.size() &&
                    col >= 0 && col < (int)unknownOccupancy[0].size())
                {
                    if (unknownOccupancy[row][col] == -1) {
                        unknownOccupancy[row][col] = 0; // wall
                        foundBy[row][col] = -2;         // discovered by vine
                    }
                }
            }
        }

        tipSensorLastUpdate = currentTime;
    }

private:
    float pendingAngleRad = 0.0f;

    void applyPendingAngleIfNeeded()
    {
        if (std::fabs(pendingAngleRad) > 1e-6f) {
            float minLen = 0.1f;
            if (length >= minLen) {
                kappa = pendingAngleRad / length;
                pendingAngleRad = 0.0f;
                //std::cout << "[DEBUG] applyPendingAngle => kappa=" << kappa << "\n";
            }
        }
    }
};

#endif // VINE_ROBOT_HPP
