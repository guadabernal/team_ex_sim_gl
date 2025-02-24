#ifndef VINE_ROBOT_HPP
#define VINE_ROBOT_HPP

#include <vector>
#include <cmath>
#include <utility>
#include <algorithm>
#include <iostream>
#include <random>

// Constants.
constexpr float PI = 3.14159265f;
constexpr float TURN_INCREMENT = 5.0f * (PI / 180.0f);
static constexpr float MIN_TURN_ANGLE = 10.0f * (PI / 180.0f); // 20°
static constexpr float MAX_TURN_ANGLE = 20.0f * (PI / 180.0f); // 45°
constexpr float COLLISION_REVERSE_DISTANCE = 1.5f;

// Global variable for the turn line for plotting.
struct TurnLine {
    std::pair<float, float> start;
    std::pair<float, float> end;
};
inline TurnLine g_turnLine = { {0, 0}, {0, 0} };

struct VineRobot {
    std::vector<std::pair<float, float>> points;
    std::default_random_engine rng;

    float theta;
    float expansionRate;

    // Turning state.
    bool turning;
    float targetTurnAngle;      // Total turn (in radians) desired in this phase
    float remainingTurnAngle;   // How much turn remains to be applied
    float turnStartHeading;     // Base heading at turn start (from last two points)
    float turnDirectionSign;    // +1 or -1; flipped on collision

    float tipSensorLastUpdate;

    // Reversal state.
    float reverseDistanceRemaining;
    bool reversing;
    int reverseCount;

    VineRobot(float startX, float startY, float initialAngleRad)
        : theta(initialAngleRad)
        , expansionRate(0.5f)
        , turning(false)
        , targetTurnAngle(0.0f)
        , remainingTurnAngle(0.0f)
        , turnStartHeading(0.0f)
        , turnDirectionSign(+1.0f)
        , tipSensorLastUpdate(0.0f)
        , reverseDistanceRemaining(0.0f)
        , reversing(false)
        , reverseCount(0)
        , rng(std::random_device{}())
    {
        points.push_back({ startX, startY });
    }

    std::pair<float, float> tip() const { return points.back(); }

    // Checks if a single point collides (using occupancy grid).
    bool isColliding(const std::pair<float, float>& pt, const std::vector<std::vector<int>>& occupancy, float cellSize) {
        int col = static_cast<int>(pt.first / cellSize);
        int row = static_cast<int>(pt.second / cellSize);
        if (row < 0 || row >= static_cast<int>(occupancy.size()) ||
            col < 0 || col >= static_cast<int>(occupancy[0].size()))
        {
            return true; // Out of bounds => collision.
        }
        return (occupancy[row][col] == 0); // 0 => wall => collision.
    }

    // NEW: Checks if any point in the vine is colliding.
    bool checkVineCollision(const std::vector<std::vector<int>>& occupancy, float cellSize) {
        for (const auto& pt : points) {
            if (isColliding(pt, occupancy, cellSize))
                return true;
        }
        return false;
    }

    void reverseOnePoint(float dist) {
        if (points.size() < 2) return; // No segment to remove.
        auto& p2 = points.back();
        auto& p1 = points[points.size() - 2];
        float dx = p2.first - p1.first;
        float dy = p2.second - p1.second;
        float segLen = std::sqrt(dx * dx + dy * dy);
        if (segLen <= dist) {
            // Remove the entire last point.
            points.pop_back();
        }
        else {
            // Shorten the last segment by 'dist'.
            float ratio = (segLen - dist) / segLen;
            p2.first = p1.first + dx * ratio;
            p2.second = p1.second + dy * ratio;
        }
    }

    float totalLength() const {
        float sum = 0.0f;
        for (size_t i = 1; i < points.size(); i++) {
            float dx = points[i].first - points[i - 1].first;
            float dy = points[i].second - points[i - 1].second;
            sum += std::sqrt(dx * dx + dy * dy);
        }
        return sum;
    }

    // -----------------------------------------
    // move() function.
    // -----------------------------------------
    void move(const std::vector<std::vector<int>>& occupancy, float cellSize, float dt) {
        // 1) Handle reversing first.
        if (reversing && reverseDistanceRemaining > 0.0f) {
            float retractStep = expansionRate * dt;
            if (retractStep > reverseDistanceRemaining) {
                retractStep = reverseDistanceRemaining;
            }
            reverseOnePoint(retractStep);
            reverseDistanceRemaining -= retractStep;
            if (reverseDistanceRemaining <= 1e-6f) {
                reversing = false;
                reverseDistanceRemaining = 0.0f;
                // Start a new turn.
                turning = true;
                std::uniform_real_distribution<float> dist(MIN_TURN_ANGLE, MAX_TURN_ANGLE);
                targetTurnAngle = dist(rng);
                remainingTurnAngle = targetTurnAngle;
                turnStartHeading = computeHeading();
                float targetHeading = turnStartHeading + turnDirectionSign * targetTurnAngle;
                float lineLength = 0.5f;
                auto tipPos = tip();
                g_turnLine.start = tipPos;
                g_turnLine.end = { tipPos.first + lineLength * std::cos(targetHeading),
                                   tipPos.second + lineLength * std::sin(targetHeading) };
                // std::cout << "Starting turn: " << (turnDirectionSign > 0 ? "right" : "left") << ", target total angle = " << targetTurnAngle * 180.0f / PI << " deg." << std::endl;
                rebalancePoints(occupancy, cellSize, dt);
                return;
            }
            rebalancePoints(occupancy, cellSize, dt);
            return;
        }

        // 2) Compute the base heading and adjust if turning.
        float baseHeading = computeHeading();
        float currentHeading = baseHeading;
        if (turning) {
            float delta = std::min(TURN_INCREMENT, remainingTurnAngle);
            currentHeading = turnStartHeading + turnDirectionSign * ((targetTurnAngle - remainingTurnAngle) + delta);
            remainingTurnAngle -= delta;
            if (remainingTurnAngle <= 1e-6f) {
                turning = false;
            }
        }
        // 3) Compute candidate point along the adjusted heading.
        float stepSize = expansionRate * dt;
        auto lastTip = tip();
        std::pair<float, float> candidate{
            lastTip.first + stepSize * std::cos(currentHeading),
            lastTip.second + stepSize * std::sin(currentHeading)
        };

        // 4) Check for collision: candidate point OR any point on the vine.
        if (isColliding(candidate, occupancy, cellSize) || checkVineCollision(occupancy, cellSize)) {
            reverseCount++;
            float desiredReverse = COLLISION_REVERSE_DISTANCE * reverseCount;
            float currentLength = totalLength();
            if (desiredReverse >= currentLength) {
                desiredReverse = currentLength;
                reverseCount = 0;
            }
            reversing = true;
            reverseDistanceRemaining = desiredReverse;
            // Flip turn direction upon collision.
            turnDirectionSign *= -1.0f;
            return;
        }

        // 5) No collision: add candidate point and gradually rebalance.
        points.push_back(candidate);
        rebalancePoints(occupancy, cellSize, dt);
    }

    // -----------------------------------------
    // sensor_lidar function.
    // -----------------------------------------
    std::vector<float> sensor_lidar(const std::vector<std::vector<int>>& occupancy, float cellSize) {
        const float max_range = 2.0f;
        float step = cellSize * 0.5f;
        std::vector<float> measurements(3, max_range);
        auto sensorPos = tip();
        float angleOffsets[3] = { -15.0f * (PI / 180.0f), 0.0f, 15.0f * (PI / 180.0f) };

        float heading = computeHeading();
        for (int i = 0; i < 3; i++) {
            float sensorAngle = heading + angleOffsets[i];
            float distance = 0.0f;
            while (distance < max_range) {
                float rayX = sensorPos.first + distance * std::cos(sensorAngle);
                float rayY = sensorPos.second + distance * std::sin(sensorAngle);
                int col = static_cast<int>(rayX / cellSize);
                int row = static_cast<int>(rayY / cellSize);
                if (row < 0 || row >= static_cast<int>(occupancy.size()) ||
                    col < 0 || col >= static_cast<int>(occupancy[0].size()))
                {
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

    // -----------------------------------------
    // measure function.
    // -----------------------------------------
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

        std::vector<float> measurements = sensor_lidar(trueOccupancy, cellSize);
        if (measurements.empty())
            return;

        auto sensorPos = tip();
        const float maxRange = 2.0f;
        float angleOffsets[3] = { -15.0f * (PI / 180.0f), 0.0f, 15.0f * (PI / 180.0f) };

        float heading = computeHeading();
        for (int i = 0; i < 3; i++) {
            float sensorAngle = heading + angleOffsets[i];
            float distance = measurements[i];
            int stepsCount = static_cast<int>(distance / cellSize);
            for (int j = 0; j < stepsCount; j++) {
                float rayX = sensorPos.first + (j * cellSize) * std::cos(sensorAngle);
                float rayY = sensorPos.second + (j * cellSize) * std::sin(sensorAngle);
                int col = static_cast<int>(rayX / cellSize);
                int row = static_cast<int>(rayY / cellSize);
                if (row >= 0 && row < static_cast<int>(unknownOccupancy.size()) &&
                    col >= 0 && col < static_cast<int>(unknownOccupancy[0].size()))
                {
                    if (unknownOccupancy[row][col] == -1) {
                        unknownOccupancy[row][col] = 1; // ground
                        foundBy[row][col] = -2;         // discovered by vine robot
                    }
                }
            }
            if (distance < maxRange) {
                float sensorX = sensorPos.first + distance * std::cos(sensorAngle);
                float sensorY = sensorPos.second + distance * std::sin(sensorAngle);
                int col = static_cast<int>(sensorX / cellSize);
                int row = static_cast<int>(sensorY / cellSize);
                if (row >= 0 && row < static_cast<int>(unknownOccupancy.size()) &&
                    col >= 0 && col < static_cast<int>(unknownOccupancy[0].size()))
                {
                    if (unknownOccupancy[row][col] == -1) {
                        unknownOccupancy[row][col] = 0; // wall
                        foundBy[row][col] = -2;         // discovered by vine robot
                    }
                }
            }
        }
        tipSensorLastUpdate = currentTime;
    }

private:
    // -----------------------------------------
    // computeHeading(): returns heading based on last two points.
    // -----------------------------------------
    float computeHeading() const {
        if (points.size() < 2)
            return theta;
        auto& p1 = points[points.size() - 2];
        auto& p2 = points.back();
        float dx = p2.first - p1.first;
        float dy = p2.second - p1.second;
        return std::atan2(dy, dx);
    }

public:

public:
    // -----------------------------------------
    // rebalancePoints(): Adjust interior points gradually.
    // Uses signed lateral offsets so left/right turns are preserved.
    // dt is used to limit how far points are moved in one call.
    // -----------------------------------------
    // Updated rebalancePoints(): use p₀ = points[0] and pₙ = points.back() as endpoints,
// and choose the circle so that p₁ is as close as possible to lying on it.
    void rebalancePoints(const std::vector<std::vector<int>>& occupancy, float cellSize, float dt) {
        if (points.size() < 3)
            return;

        // Use p₀ (first point) and pₙ (last point) to define the chord.
        const auto& p0 = points[0];
        const auto& pN = points.back();
        float dx = pN.first - p0.first;
        float dy = pN.second - p0.second;
        float L = std::sqrt(dx * dx + dy * dy);
        if (L < 1e-6f)
            return; // Chord too short.

        // Midpoint and unit vector along the chord.
        std::pair<float, float> M = { (p0.first + pN.first) / 2.0f, (p0.second + pN.second) / 2.0f };
        std::pair<float, float> u = { dx / L, dy / L };

        // Perpendicular unit vector (pointing to the right).
        std::pair<float, float> n = { -u.second, u.first };

        // Use the second point (p₁) to choose the circle from the family of circles through p₀ and pₙ.
        // Let A = p₁ - M.
        const auto& p1 = points[1];
        float Ax = p1.first - M.first;
        float Ay = p1.second - M.second;
        float A_norm_sq = Ax * Ax + Ay * Ay;
        float halfL_sq = (L / 2.0f) * (L / 2.0f);
        // p₁ lies exactly on the circle if: |A - t·n| = sqrt(halfL_sq + t²) (which is R)
        // Solving for t yields: t = (|A|² - (L/2)²) / (2*(A·n)).
        float dotAn = Ax * n.first + Ay * n.second;
        float t = 0.0f;
        if (std::fabs(dotAn) > 1e-6f)
            t = (A_norm_sq - halfL_sq) / (2.0f * dotAn);
        else
            t = 0.0f; // If p₁ is nearly on the perpendicular bisector, choose center = M.

        // Circle center and radius.
        std::pair<float, float> C = { M.first + t * n.first, M.second + t * n.second };
        float R = std::sqrt(halfL_sq + t * t);

        // Compute angles from center to endpoints.
        float angle0 = std::atan2(p0.second - C.second, p0.first - C.first);
        float angleN = std::atan2(pN.second - C.second, pN.first - C.first);
        // Compute the smallest angular difference in [-PI, PI].
        float dAngle = angleN - angle0;
        while (dAngle > PI) dAngle -= 2.0f * PI;
        while (dAngle < -PI) dAngle += 2.0f * PI;

        // Compute target positions along the arc.
        // p₀ and pₙ are fixed; interior points (indices 1 .. n–1) will be moved gradually.
        std::vector<std::pair<float, float>> targetPoints(points.size());
        targetPoints[0] = p0;
        targetPoints[points.size() - 1] = pN;
        for (size_t i = 1; i < points.size() - 1; i++) {
            float fraction = static_cast<float>(i) / (points.size() - 1);
            float targetAngle = angle0 + fraction * dAngle;
            targetPoints[i] = { C.first + R * std::cos(targetAngle),
                                C.second + R * std::sin(targetAngle) };
        }

        // Gradually move each interior point toward its target.
        float allowedStep = expansionRate * dt;
        for (size_t i = 1; i < points.size() - 1; i++) {
            float diffX = targetPoints[i].first - points[i].first;
            float diffY = targetPoints[i].second - points[i].second;
            float dist = std::sqrt(diffX * diffX + diffY * diffY);
            if (dist <= allowedStep || dist < 1e-6f)
                points[i] = targetPoints[i];
            else {
                float ratio = allowedStep / dist;
                points[i].first += diffX * ratio;
                points[i].second += diffY * ratio;
            }
        }
    }

};

#endif
