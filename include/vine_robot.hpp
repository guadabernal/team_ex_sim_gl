#pragma once

#include <vector>
#include <cmath>
#include <utility>
#include <algorithm>
#include <iostream>

constexpr float PI = 3.14159265f;
constexpr float TURN_INCREMENT = 10.0f * (PI / 180.0f);

float targetTurnAngle = 0.0f;  // total angle for the current turn (radians)
static constexpr float MIN_TURN_ANGLE = 20.0f * (PI / 180.0f); // 15°
static constexpr float MAX_TURN_ANGLE = 45.0f * (PI / 180.0f); // 60°

constexpr float COLLISION_REVERSE_DISTANCE = 1.5f;

struct VineRobot
{
    std::vector<std::pair<float, float>> points;
    std::default_random_engine rng;

    float theta;
    float expansionRate;

    bool  turning;
    int   stepsLeftInTurn;
    float turnDirectionSign;
    float tipSensorLastUpdate;

    float reverseDistanceRemaining;
    bool  reversing;
    int   reverseCount;

    VineRobot(float startX, float startY, float initialAngleRad)
        : theta(initialAngleRad)
        , expansionRate(0.05f)
        , turning(false)
        , stepsLeftInTurn(0)
        , turnDirectionSign(+1.0f)
        , reverseDistanceRemaining(0.0f)
        , reversing(false)
        , tipSensorLastUpdate(0.0f)
        , reverseCount(0)
        , rng(std::random_device{}())
    {
        points.push_back({ startX, startY });
    }

    std::pair<float, float> tip() const { return points.back(); }
    bool isColliding(const std::pair<float, float>& pt, const std::vector<std::vector<int>>& occupancy, float cellSize)
    {
        int col = static_cast<int>(pt.first / cellSize);
        int row = static_cast<int>(pt.second / cellSize);
        if (row < 0 || row >= (int)occupancy.size() ||
            col < 0 || col >= (int)occupancy[0].size())
        {
            return true; // out of bounds => collision
        }
        return (occupancy[row][col] == 0); // 0 => wall => collision
    }
    void reverseOnePoint(float dist)
    {
        if (points.size() < 2) return; // no segment to remove
        auto& p2 = points.back();
        auto& p1 = points[points.size() - 2];

        float dx = p2.first - p1.first;
        float dy = p2.second - p1.second;
        float segLen = std::sqrt(dx * dx + dy * dy);

        if (segLen <= dist) {
            // Remove that entire last point
            points.pop_back();
        }
        else {
            // Partially shorten this segment by 'dist'
            float ratio = (segLen - dist) / segLen;
            p2.first = p1.first + dx * ratio;
            p2.second = p1.second + dy * ratio;
        }
    }
    float totalLength() const {
        float sum = 0.0f;
        for (int i = 1; i < (int)points.size(); i++) {
            float dx = points[i].first - points[i - 1].first;
            float dy = points[i].second - points[i - 1].second;
            sum += std::sqrt(dx * dx + dy * dy);
        }
        return sum;
    }

    // -----------------------------------------
    // move() function
    // -----------------------------------------
    /*void move(const std::vector<std::vector<int>>& occupancy, float cellSize, float dt)
    {
        // 1) If we are currently reversing, handle that first
        //    We'll do as much retraction as expansionRate*dt, but only one segment at a time
        if (reversing && reverseDistanceRemaining > 0.0f)
        {
            float retractStep = expansionRate * dt;
            if (retractStep > reverseDistanceRemaining) {
                retractStep = reverseDistanceRemaining;
            }
            reverseOnePoint(retractStep);
            reverseDistanceRemaining -= retractStep;

            // If we've finished reversing 10 cm, switch to turn
            if (reverseDistanceRemaining <= 1e-6f) {
                reversing = false;
                reverseDistanceRemaining = 0.0f;

                // Start a new 5-step turn
                turning = true;
                stepsLeftInTurn = 100;
                std::cout << "turning :";
                if(turnDirectionSign > 0) std::cout << " right" <<std::endl;
                else std::cout << " left" << std::endl;
                // We skip normal forward expansion this frame
            }
            rebalancePoints(occupancy, cellSize);
            return;
        }

        // 2) If turning => add turn increment
        float currentAngle = computeHeading();
        float turnAmount = 0.0f;
        if (turning && stepsLeftInTurn > 0) {
            turnAmount = TURN_INCREMENT * turnDirectionSign;
            stepsLeftInTurn--;
            if (stepsLeftInTurn == 0) {
                std::cout << "Done Turning" << std::endl;
                turning = false;
            }
        }
        float newAngle = currentAngle + turnAmount;

        // Optionally clamp ±30°
        float diff = newAngle - theta;
        if (diff > MAX_ANGLE_RANGE) newAngle = theta + MAX_ANGLE_RANGE;
        if (diff < -MAX_ANGLE_RANGE) newAngle = theta - MAX_ANGLE_RANGE;

        // 3) Propose a new forward step
        float stepSize = expansionRate * dt;
        auto  lastTip = tip();
        std::pair<float, float> candidate{
            lastTip.first + stepSize * std::cos(newAngle),
            lastTip.second + stepSize * std::sin(newAngle)
        };

        // 4) Check collision
        if (!isColliding(candidate, occupancy, cellSize)) {
            // no collision => add new point
            points.push_back(candidate);
        }
        else {
            reverseCount++;
            float desiredReverse = COLLISION_REVERSE_DISTANCE * reverseCount;

            float currentLength = totalLength();
            if (desiredReverse >= currentLength) {
                desiredReverse = currentLength;
            }

            reversing = true;
            reverseDistanceRemaining = desiredReverse;
            turnDirectionSign *= -1.0f;
            return;
        }

        // 5) Rebalance
        rebalancePoints(occupancy, cellSize);
    }*/

    void move(const std::vector<std::vector<int>>& occupancy, float cellSize, float dt) {
        // 1) If reversing, handle that first.
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
                // Turn is triggered when reverse finishes.
                turning = true;
                // Randomly select a total turn angle between MIN_TURN_ANGLE and MAX_TURN_ANGLE.
                // (Assuming you have a random number generator, e.g., std::default_random_engine rng;)
                std::uniform_real_distribution<float> dist(MIN_TURN_ANGLE, MAX_TURN_ANGLE);
                targetTurnAngle = dist(rng);
                // Determine the number of steps needed based on a fixed per-step increment.
                stepsLeftInTurn = static_cast<int>(std::ceil(targetTurnAngle / TURN_INCREMENT));
                std::cout << "Starting turn: " << (turnDirectionSign > 0 ? "right" : "left")
                          << ", target total angle = " << targetTurnAngle * 180.0f / PI << " deg in " << stepsLeftInTurn << " steps." << std::endl;
                // Skip forward expansion this frame.
                rebalancePoints(occupancy, cellSize);
                return;
            }
            rebalancePoints(occupancy, cellSize);
            return;
        }

        // 2) If turning, apply the fixed per-step turn increment.
        float currentAngle = computeHeading(); // Derived from the last two vine points.
        float turnAmount = 0.0f;
        if (turning && stepsLeftInTurn > 0) {
            // For simplicity, each step adds TURN_INCREMENT (and the last step may add a smaller remainder)
            if (stepsLeftInTurn == 1) {
                // Compute the exact remaining angle.
                float alreadyTurned = targetTurnAngle - (stepsLeftInTurn * TURN_INCREMENT);
                turnAmount = targetTurnAngle - alreadyTurned;
            }
            else {
                turnAmount = TURN_INCREMENT;
            }
            turnAmount *= turnDirectionSign;
            // Use the modified angle as the new heading.
            currentAngle += turnAmount;
            stepsLeftInTurn--;
            if (stepsLeftInTurn <= 0) {
                turning = false;
                //std::cout << "Done Turning" << std::endl;
            }
        }

        // 3) Propose a new forward step using currentAngle.
        float stepSize = expansionRate * dt;
        auto lastTip = tip();
        std::pair<float, float> candidate{
             lastTip.first + stepSize * std::cos(currentAngle),
             lastTip.second + stepSize * std::sin(currentAngle)
        };

        // 4) Check collision.
        if (!isColliding(candidate, occupancy, cellSize)) {
            points.push_back(candidate);
        }
        else {
            reverseCount++;
            float desiredReverse = COLLISION_REVERSE_DISTANCE * reverseCount;
            float currentLength = totalLength();
            if (desiredReverse >= currentLength) {
                desiredReverse = currentLength;
                reverseCount = 0;
            }
            reversing = true;
            reverseDistanceRemaining = desiredReverse;
            // Flip turn direction for next turn.
            turnDirectionSign *= -1.0f;
            return;
        }

        // 5) Rebalance the vine.
        rebalancePoints(occupancy, cellSize);
    }


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
        // Sensor offsets in radians: -15°, 0°, +15°
        float angleOffsets[3] = { -15.0f * (PI / 180.0f), 0.0f, 15.0f * (PI / 180.0f) };

        // Compute the heading from the last two vine points.
        float heading = computeHeading();

        for (int i = 0; i < 3; i++) {
            // Use the computed heading instead of theta.
            float sensorAngle = heading + angleOffsets[i];
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
                        foundBy[row][col] = -2;         // discovered by vine robot
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
                        foundBy[row][col] = -2;         // discovered by vine robot
                    }
                }
            }
        }
        tipSensorLastUpdate = currentTime;
    }


private:
    // -----------------------------------------
    // Simple heading from the last two points
    // -----------------------------------------
    float computeHeading() const
    {
        if (points.size() < 2) {
            return theta;
        }
        auto& p1 = points[points.size() - 2];
        auto& p2 = points.back();
        float dx = p2.first - p1.first;
        float dy = p2.second - p1.second;
        return std::atan2(dy, dx);
    }

    // -----------------------------------------
    //  WORKING VERSION
    // -----------------------------------------
    /*void rebalancePoints_0()
    {
        // If too few points, nothing to do
        if (points.size() < 3) return;

        // 1) Chord from p0->pN
        auto p0 = points.front();
        auto pN = points.back();
        float dx = pN.first - p0.first;
        float dy = pN.second - p0.second;
        float L = std::sqrt(dx * dx + dy * dy);
        if (L < 1e-8f) return; // degenerate

        // Unit chord direction
        float ux = dx / L;
        float uy = dy / L;

        // 2) Compute old area between chord and existing shape
        //    We'll do a trapezoid rule over each segment i->(i+1)
        //    based on projection 't_i' and offset 'offset_i'.
        //    Because p0 and pN are on chord, offset_0=offset_{N-1}=0.
        std::vector<float> tvals(points.size(), 0.0f);
        std::vector<float> offsets(points.size(), 0.0f);

        // We'll store the projection (t_i) and offset for each point
        for (int i = 0; i < (int)points.size(); i++) {
            float rx = points[i].first - p0.first;
            float ry = points[i].second - p0.second;
            float proj = rx * ux + ry * uy;  // distance along chord from p0
            tvals[i] = proj;

            // perpendicular offset
            //  projection point on chord
            float px = p0.first + proj * ux;
            float py = p0.second + proj * uy;
            float ox = points[i].first - px;
            float oy = points[i].second - py;
            float dist = std::sqrt(ox * ox + oy * oy);
            // You can do sign if you want, but let's take absolute offset for area
            offsets[i] = dist;
        }

        // Trapezoid integration
        // offset_0=0, offset_{N-1}=0 if p0,pN are exactly on chord
        float oldArea = 0.0f;
        for (int i = 0; i < (int)points.size() - 1; i++) {
            float base = (tvals[i + 1] - tvals[i]);    // horizontal delta
            float avgH = 0.5f * (offsets[i] + offsets[i + 1]);
            oldArea += (base * avgH);
        }

        // If oldArea is near 0, no need to bend
        if (oldArea < 1e-8f) {
            return;
        }

        // 3) Solve for "segment height" h so that the circle-segment area = oldArea.
        //    circle-seg area formula:
        //       R = (h^2 + (L^2)/4) / (2h)
        //       A_segment = R^2 * acos((R-h)/R) - (R - h) * sqrt(2R h - h^2)
        //    We'll do a small binary search for h in [0..someMax].
        auto segmentArea = [&](float h) {
            // radius
            float R = (h * h + 0.25f * L * L) / (2.0f * h);
            // angle approach: or direct formula
            // A = R^2 * acos((R-h)/R) - (R-h)*sqrt(2Rh - h^2)
            float top = (R - h);
            float inside = (2.f * R * h - h * h);
            if (inside < 0.f) return 0.f; // invalid geometry
            float A = R * R * std::acos(top / R) - top * std::sqrt(inside);
            return A;
            };

        // We'll define a small helper to clamp or handle edge cases
        float minH = 1e-6f;      // can't be 0 or negative
        float maxH = L;          // a large upper bound
        // If oldArea is bigger than the area of a semicircle, you might need > L, 
        // but typically this is enough.

        float target = oldArea;
        float bestH = 0.0f;
        for (int iter = 0; iter < 50; iter++) {
            float midH = 0.5f * (minH + maxH);
            float A = segmentArea(midH);
            if (std::fabs(A - target) < 1e-6f) {
                bestH = midH;
                break;
            }
            if (A > target) {
                maxH = midH;
            }
            else {
                minH = midH;
            }
            bestH = midH;
        }

        // Now we have bestH => The circle's segment has ~the same area as oldArea.

        // 4) Re-sample points along that new arc
        //    a) Compute radius R, arc angle phi
        float R = (bestH * bestH + 0.25f * L * L) / (2.0f * bestH);
        // The chord is L, so half-chord is L/2 => angle:
        float halfChordOverR = (0.5f * L) / R;
        // Safety clamp if halfChordOverR ~> 1
        if (halfChordOverR > 1.f) halfChordOverR = 1.f;
        float alpha = std::asin(halfChordOverR);
        float phi = 2.0f * alpha;        // total arc angle
        float arcLen = R * phi;

        //  b) We'll place interior points at equal arc distances s_i
        //     from 0..arcLen, with s_0=0 => p0, s_{N-1}=arcLen => pN
        //  c) We'll define a local coordinate system:
        //     - x-axis = chord direction (u_x,u_y)
        //     - y-axis = normal direction ( -u_y, u_x ) if we want "above" chord
        //  d) We must find the circle center. For a segment height h above chord,
        //     the center is at distance "R - h" from the chord's near side.
        //     If we want the arc "above" chord, centerY is + (R - h).

        float nx = -uy;
        float ny = ux;


        // center is at p0 + "some offset" along chord + normal
        // The midpoint of the chord is pMid = p0 + 0.5f*L*(u_x,u_y).
        float midX = p0.first + 0.5f * L * ux;
        float midY = p0.second + 0.5f * L * uy;
        // "arc sag" from the midpoint is (R - h) in the normal direction
        // so center = pMid + (R - h)*(nx, ny)
        float centerX = midX + (R - bestH) * nx;
        float centerY = midY + (R - bestH) * ny;

        // We want the starting angle so that p0 is on the arc. 
        // Vector from center->p0
        float c0x = p0.first - centerX;
        float c0y = p0.second - centerY;
        float startAngle = std::atan2(c0y, c0x);
        // arc direction is +phi from start to end => for i in [0..N-1]
        // Each step => angle = startAngle + (s_i / R)

        // Overwrite points with the new arc positions
        points[0] = p0;     // ensure p0 is exact
        points[points.size() - 1] = pN; // ensure pN is exact

        for (int i = 1; i < (int)points.size() - 1; i++) {
            float s_i = (arcLen) * ((float)i / (points.size() - 1));
            float angle_i = startAngle + (s_i / R);

            float arcX = centerX + R * std::cos(angle_i);
            float arcY = centerY + R * std::sin(angle_i);
            points[i] = std::make_pair(arcX, arcY);
        }
    }*/


    void rebalancePoints(const std::vector<std::vector<int>>& occupancy, float cellSize)
    {
        // If too few points, nothing to do.
        if (points.size() < 3) return;

        // 1) Compute chord from first point (p0) to tip (pN)
        auto p0 = points.front();
        auto pN = points.back();
        float dx = pN.first - p0.first;
        float dy = pN.second - p0.second;
        float L = std::sqrt(dx * dx + dy * dy);
        if (L < 1e-8f) return; // degenerate

        // Unit chord direction.
        float ux = dx / L;
        float uy = dy / L;

        // 2) Compute the "old" area between the current vine and the chord.
        std::vector<float> tvals(points.size(), 0.0f);
        std::vector<float> offsets(points.size(), 0.0f);
        for (int i = 0; i < (int)points.size(); i++) {
            float rx = points[i].first - p0.first;
            float ry = points[i].second - p0.second;
            float proj = rx * ux + ry * uy;
            tvals[i] = proj;
            float px = p0.first + proj * ux;
            float py = p0.second + proj * uy;
            float ox = points[i].first - px;
            float oy = points[i].second - py;
            offsets[i] = std::sqrt(ox * ox + oy * oy);
        }
        float oldArea = 0.0f;
        for (int i = 0; i < (int)points.size() - 1; i++) {
            float base = tvals[i + 1] - tvals[i];
            float avgH = 0.5f * (offsets[i] + offsets[i + 1]);
            oldArea += base * avgH;
        }
        if (oldArea < 1e-8f) return; // no bending needed

        // 3) Solve for the segment height h such that the circle segment area equals oldArea.
        auto segmentArea = [&](float h) -> float {
            float R = (h * h + 0.25f * L * L) / (2.0f * h);
            float top = (R - h);
            float inside = (2.f * R * h - h * h);
            if (inside < 0.f) return 0.f;
            float A = R * R * std::acos(top / R) - top * std::sqrt(inside);
            return A;
            };
        float minH = 1e-6f, maxH = L;
        float target = oldArea;
        float bestH = 0.0f;
        for (int iter = 0; iter < 50; iter++) {
            float midH = 0.5f * (minH + maxH);
            float A = segmentArea(midH);
            if (std::fabs(A - target) < 1e-6f) {
                bestH = midH;
                break;
            }
            if (A > target)
                maxH = midH;
            else
                minH = midH;
            bestH = midH;
        }

        // 4) Compute circle parameters.
        float R_val = (bestH * bestH + 0.25f * L * L) / (2.0f * bestH);
        float halfChordOverR = (0.5f * L) / R_val;
        if (halfChordOverR > 1.f) halfChordOverR = 1.f;
        float alpha = std::asin(halfChordOverR);
        float phi = 2.0f * alpha;        // total arc angle
        float arcLen = R_val * phi;

        // 5) Define local coordinate system and compute circle center.
        // Here we use normal (nx, ny) = (-uy, ux) as before.
        float nx = -uy;
        float ny = ux;
        float midX = p0.first + 0.5f * L * ux;
        float midY = p0.second + 0.5f * L * uy;
        // For this version we assume no change in turn direction;
        // the center is computed solely from the current chord.
        float centerX = midX + (R_val - bestH) * nx;
        float centerY = midY + (R_val - bestH) * ny;

        // 6) Determine starting angle so that p0 lies on the arc.
        float c0x = p0.first - centerX;
        float c0y = p0.second - centerY;
        float startAngle = std::atan2(c0y, c0x);

        // 7) Compute target arc positions for all intermediate points.
        std::vector<std::pair<float, float>> targetPoints(points.size());
        targetPoints[0] = p0;
        targetPoints[points.size() - 1] = pN;
        for (int i = 1; i < (int)points.size() - 1; i++) {
            float s_i = arcLen * ((float)i / ((float)points.size() - 1));
            float angle_i = startAngle + (s_i / R_val);
            float arcX = centerX + R_val * std::cos(angle_i);
            float arcY = centerY + R_val * std::sin(angle_i);
            targetPoints[i] = std::make_pair(arcX, arcY);
        }

        // 8) Check for collisions along the vine (excluding the tip).
        // We assume a helper function isColliding(point, occupancy, cellSize) is available.
        // Find the highest index (pivot) that is colliding.
        int pivotIndex = -1;
        for (int i = 0; i < (int)points.size() - 1; i++) {
            if (isColliding(points[i], occupancy, cellSize)) {
                pivotIndex = i; // update pivot to the latest collision point
            }
        }

        // 9) Update points:
        // If a collision is detected (pivotIndex != -1),
        // then freeze all points up to (and including) pivotIndex, and update only points after it.
        if (pivotIndex != -1) {
            // Leave points[0..pivotIndex] unchanged.
            for (int i = pivotIndex + 1; i < (int)points.size() - 1; i++) {
                points[i] = targetPoints[i];
            }
        }
        else {
            // No collision: update all intermediate points.
            for (int i = 1; i < (int)points.size() - 1; i++) {
                points[i] = targetPoints[i];
            }
        }
    }


    // kinda works in both directions version 
    /*void rebalancePoints_1() {
        if (points.size() < 3) return;
        // 1) Compute the chord from p0 to pN.
        auto p0 = points.front();
        auto pN = points.back();
        float dx = pN.first - p0.first;
        float dy = pN.second - p0.second;
        float L = std::sqrt(dx * dx + dy * dy);
        if (L < 1e-8f) return; // degenerate case
        float ux = dx / L, uy = dy / L;
        // 2) Compute the area between the vine and the chord.
        std::vector<float> tvals(points.size(), 0.0f);
        std::vector<float> offsets(points.size(), 0.0f);
        for (size_t i = 0; i < points.size(); i++) {
            float rx = points[i].first - p0.first;
            float ry = points[i].second - p0.second;
            float proj = rx * ux + ry * uy;
            tvals[i] = proj;
            float px = p0.first + proj * ux;
            float py = p0.second + proj * uy;
            float ox = points[i].first - px;
            float oy = points[i].second - py;
            offsets[i] = std::sqrt(ox * ox + oy * oy);
        }
        float oldArea = 0.0f;
        for (size_t i = 0; i < points.size() - 1; i++) {
            float base = tvals[i + 1] - tvals[i];
            float avgH = 0.5f * (offsets[i] + offsets[i + 1]);
            oldArea += base * avgH;
        }
        if (oldArea < 1e-8f) return; // no significant bending
        // 3) Solve for the segment height h such that the area of the circle segment equals oldArea.
        auto segmentArea = [&](float h) -> float {
            float R = (h * h + 0.25f * L * L) / (2.0f * h);
            float top = R - h;
            float inside = 2.0f * R * h - h * h;
            if (inside < 0.f) return 0.f;
            return R * R * std::acos(top / R) - top * std::sqrt(inside);
            };
        float minH = 1e-6f, maxH = L, bestH = 0.0f;
        for (int iter = 0; iter < 50; iter++) {
            float midH = 0.5f * (minH + maxH);
            float A = segmentArea(midH);
            if (std::fabs(A - oldArea) < 1e-6f) {
                bestH = midH;
                break;
            }
            if (A > oldArea)
                maxH = midH;
            else
                minH = midH;
            bestH = midH;
        }
        // 4) Compute circle parameters.
        float R = (bestH * bestH + 0.25f * L * L) / (2.0f * bestH);
        float halfChordOverR = (0.5f * L) / R;
        if (halfChordOverR > 1.f) halfChordOverR = 1.f;
        float alphaAngle = std::asin(halfChordOverR);
        float phi = 2.0f * alphaAngle; // total arc angle
        float arcLen = R * phi;
        // 5) Compute the circle center.
        float midX = p0.first + 0.5f * L * ux;
        float midY = p0.second + 0.5f * L * uy;
        // Standard normal to the chord is (-uy, ux)
        float nx = -uy;
        float ny = ux;
        // Use turnDirectionSign to choose the correct side.
        float centerX = midX + (R - bestH) * (turnDirectionSign * nx);
        float centerY = midY + (R - bestH) * (turnDirectionSign * ny);
        // 6) Determine the starting angle from the center to p0.
        float c0x = p0.first - centerX;
        float c0y = p0.second - centerY;
        float startAngle = std::atan2(c0y, c0x);
        // 7) Compute target positions for each intermediate point along the arc.
        std::vector<std::pair<float, float>> targetPoints(points.size());
        targetPoints.front() = p0;
        targetPoints.back() = pN;
        for (size_t i = 1; i < points.size() - 1; i++) {
            float s_i = arcLen * (static_cast<float>(i) / (points.size() - 1));
            // Increment angle by turnDirectionSign*(s_i/R) to trace in the proper direction.
            float angle_i = startAngle + turnDirectionSign * (s_i / R);
            float targetX = centerX + R * std::cos(angle_i);
            float targetY = centerY + R * std::sin(angle_i);
            targetPoints[i] = { targetX, targetY };
        }
        // 8) Gradually update intermediate points toward the target positions.
        float interpAlpha = 0.05f; // Adjust this factor to control how fast points shift.
        for (size_t i = 1; i < points.size() - 1; i++) {
            points[i].first += interpAlpha * (targetPoints[i].first - points[i].first);
            points[i].second += interpAlpha * (targetPoints[i].second - points[i].second);
        }
    }*/


};
