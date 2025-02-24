#pragma once
#include <simul.hpp>
#include <GLFW/glfw3.h>
#include <cmath>
#include <vector>
#include <vine_robot.hpp>

#include <algorithm>
#include <iomanip>

inline void renderRobots(const Simulation& simulation, float scaleFactor) {
    for (const auto& robot : simulation.rr) {
        if (!robot.spawned || robot.dead && robot.battery > 0)
            continue;

        // Convert world coordinates (meters) to screen coordinates.
        float sx = robot.x * scaleFactor;
        float sy = robot.y * scaleFactor;

        float size = robot.size;

        // Define triangle vertices in the robot's local coordinate system.
        // The triangle points in the +x direction.
        float tipX = size / 2.0f;      // Tip (front)
        float tipY = 0.0f;
        float baseLeftX = -size / 2.0f;     // Rear left
        float baseLeftY = -size / 2.0f;
        float baseRightX = -size / 2.0f;     // Rear right
        float baseRightY = size / 2.0f;

        // Rotate the vertices by robot.theta.
        float cosTheta = cosf(robot.theta);
        float sinTheta = sinf(robot.theta);
        float rTipX = tipX * cosTheta - tipY * sinTheta;
        float rTipY = tipX * sinTheta + tipY * cosTheta;
        float rBaseLeftX = baseLeftX * cosTheta - baseLeftY * sinTheta;
        float rBaseLeftY = baseLeftX * sinTheta + baseLeftY * cosTheta;
        float rBaseRightX = baseRightX * cosTheta - baseRightY * sinTheta;
        float rBaseRightY = baseRightX * sinTheta + baseRightY * cosTheta;

        // Translate vertices by the robot's screen position.
        // Multiply the displacements by scaleFactor to convert meters to pixels.
        float v0x = sx + rTipX * scaleFactor;
        float v0y = sy + rTipY * scaleFactor;
        float v1x = sx + rBaseLeftX * scaleFactor;
        float v1y = sy + rBaseLeftY * scaleFactor;
        float v2x = sx + rBaseRightX * scaleFactor;
        float v2y = sy + rBaseRightY * scaleFactor;

        //glColor3f(1.0f, 1.0f, 0.0f); // yellow
        glColor3f(0.5f, 0.0f, 0.0f); // dark red

        glBegin(GL_TRIANGLES);
        glVertex2f(v0x, v0y);
        glVertex2f(v1x, v1y);
        glVertex2f(v2x, v2y);
        glEnd();
    }
}

inline void renderGrid(const Simulation& simulation, float scaleFactor) {
    int gridRows = simulation.known_grid.occupancy.size();
    int gridCols = simulation.known_grid.occupancy[0].size();
    float cellSize = simulation.known_grid.cellSize * scaleFactor;
    for (int y = 0; y < gridRows; ++y) {
        for (int x = 0; x < gridCols; ++x) {
            int cell = simulation.known_grid.occupancy[y][x];
            switch (cell) {
            case 0: // Wall -> black
                glColor3f(0.0f, 0.0f, 0.0f);
                break;
            case 1: // Ground -> white
                glColor3f(1.0f, 1.0f, 1.0f);
                break;
            case 2: // Hole -> dark blue
                glColor3f(0.0f, 0.0f, 0.5f);
                break;
            default:
                glColor3f(1.0f, 1.0f, 1.0f);
                break;
            }
            float left = x * cellSize;
            float top = y * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;
            glBegin(GL_QUADS);
            glVertex2f(left, top);
            glVertex2f(right, top);
            glVertex2f(right, bottom);
            glVertex2f(left, bottom);
            glEnd();
        }
    }
}

inline void renderMeasurementGrid(const Simulation& simulation, float scaleFactor) {
    int gridRows = simulation.grid.occupancy.size();
    int gridCols = simulation.grid.occupancy[0].size();
    float cellSize = simulation.known_grid.cellSize * scaleFactor;
    for (int y = 0; y < gridRows; ++y) {
        for (int x = 0; x < gridCols; ++x) {
            int cell = simulation.grid.occupancy[y][x];
            // If cell is unexplored (-1), use grey.
            if (cell == -1) {
                glColor3f(0.5f, 0.5f, 0.5f); // grey
            }
            else {
                // Otherwise, color as follows:
                // 0: wall -> black
                // 1: ground -> white
                // 2: hole -> dark blue
                switch (cell) {
                case 0:
                    glColor3f(0.0f, 0.0f, 0.0f); // black
                    break;
                case 1:
                    glColor3f(1.0f, 1.0f, 1.0f); // white
                    break;
                case 2:
                    glColor3f(0.0f, 0.0f, 0.5f); // dark blue
                    break;
                default:
                    glColor3f(1.0f, 1.0f, 1.0f);
                    break;
                }
            }
            float left = x * cellSize;
            float top = y * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;
            glBegin(GL_QUADS);
            glVertex2f(left, top);
            glVertex2f(right, top);
            glVertex2f(right, bottom);
            glVertex2f(left, bottom);
            glEnd();
        }
    }
}

inline void renderHeatMap(const Simulation& simulation, float scaleFactor) {
    int rows = simulation.known_grid.heat.size();
    if (rows == 0) return;
    int cols = simulation.known_grid.heat[0].size();
    float cellSize = simulation.known_grid.cellSize * scaleFactor;

    // Define the temperature range.
    const float baseTemp = 20.0f;  // Light blue at 20°C.
    const float maxTemp = 37.0f;   // Full red at 37°C.

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float temp = simulation.known_grid.heat[i][j];
            float norm = (temp - baseTemp) / (maxTemp - baseTemp);
            if (norm < 0.0f) norm = 0.0f;
            if (norm > 1.0f) norm = 1.0f;
            // Linear interpolation: light blue (0.5,0.5,1.0) -> red (1.0,0.0,0.0).
            float r = 0.8f + 0.8f * norm;
            float g = 0.8f - 0.8f * norm;
            float b = 1.0f - norm;
            glColor3f(r, g, b);

            float left = j * cellSize;
            float top = i * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;
            glBegin(GL_QUADS);
            glVertex2f(left, top);
            glVertex2f(right, top);
            glVertex2f(right, bottom);
            glVertex2f(left, bottom);
            glEnd();
        }
    }

    int occRows = simulation.known_grid.occupancy.size();
    int occCols = simulation.known_grid.occupancy[0].size();
    for (int y = 0; y < occRows; y++) {
        for (int x = 0; x < occCols; x++) {
            int cell = simulation.known_grid.occupancy[y][x];
            if (cell == 0) { // Wall
                glColor3f(0.0f, 0.0f, 0.0f);
            }
            else if (cell == 2) { // Hole
                glColor3f(0.0f, 0.0f, 0.5f);
            }
            else {
                continue; // No overlay for ground (value 1)
            }
            float left = x * cellSize;
            float top = y * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;
            glBegin(GL_QUADS);
            glVertex2f(left, top);
            glVertex2f(right, top);
            glVertex2f(right, bottom);
            glVertex2f(left, bottom);
            glEnd();
        }
    }
    float cellSizePixels = simulation.known_grid.cellSize * scaleFactor;
    for (const auto& pos : simulation.personPositions) {
        // 'pos' is in grid coordinates, so add 0.5 to center in the cell.
        float centerX = (pos.first / simulation.known_grid.cellSize + 0.5f) * cellSizePixels;
        float centerY = (pos.second / simulation.known_grid.cellSize + 0.5f) * cellSizePixels;
        // Convert the person's physical radius (in meters) to screen pixels.
        float radiusScreen = 0.15f * scaleFactor;

        // Draw the circle outline around the person.
        glColor3f(0.0f, 0.0f, 0.0f); // Black
        glLineWidth(2.0f);
        const int numSegments = 50;
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < numSegments; i++) {
            float angle = 2.0f * 3.14159265f * i / numSegments;
            float x = centerX + radiusScreen * std::cos(angle);
            float y = centerY + radiusScreen * std::sin(angle);
            glVertex2f(x, y);
        }
        glEnd();

        // Draw an X at the center of the heat source.
        float halfXSize = radiusScreen * 0.5f;
        glBegin(GL_LINES);
        glVertex2f(centerX - halfXSize, centerY - halfXSize);
        glVertex2f(centerX + halfXSize, centerY + halfXSize);
        glVertex2f(centerX - halfXSize, centerY + halfXSize);
        glVertex2f(centerX + halfXSize, centerY - halfXSize);
        glEnd();
    }
}

inline void renderDiscoveredHeatMap(const Simulation& simulation, float scaleFactor) {
    int rows = simulation.grid.heat.size();
    if (rows == 0) return;
    int cols = simulation.grid.heat[0].size();
    float cellSize = simulation.known_grid.cellSize * scaleFactor;

    // Temperature parameters.
    const float baseTemp = 20.0f;  // Low measured temperature (for mapping)
    const float maxTemp = 37.0f;   // High measured temperature (for mapping)

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // First, check occupancy.
            if (simulation.grid.occupancy[i][j] == -1) {
                // Cell not discovered by the distance sensor: render as gray.
                glColor3f(0.5f, 0.5f, 0.5f);
            }
            else if (simulation.grid.occupancy[i][j] == 0) {
                // Discovered wall: render as black.
                glColor3f(0.0f, 0.0f, 0.0f);
            }
            else if (simulation.grid.occupancy[i][j] == 1) {
                // Discovered ground cell.
                float temp = simulation.grid.heat[i][j];
                if (temp == 0.0f) {
                    // If the cell hasn't been updated by the heat sensor, render as white.
                    glColor3f(1.0f, 1.0f, 1.0f);
                }
                else {
                    // Otherwise, map the measured temperature from light blue to red.
                    float norm = (temp - baseTemp) / (maxTemp - baseTemp);
                    if (norm < 0.0f) norm = 0.0f;
                    if (norm > 1.0f) norm = 1.0f;
                    // At norm==0: light blue (0.5, 0.5, 1.0), at norm==1: red (1.0, 0.0, 0.0)
                    float r = 0.8f + 0.8f * norm;
                    float g = 0.8f - 0.8f * norm;
                    float b = 1.0f - norm;
                    glColor3f(r, g, b);
                }
            }

            float left = j * cellSize;
            float top = i * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;
            glBegin(GL_QUADS);
            glVertex2f(left, top);
            glVertex2f(right, top);
            glVertex2f(right, bottom);
            glVertex2f(left, bottom);
            glEnd();
        }
    }

    // Optionally, overlay discovered walls to reinforce boundaries.
    int occRows = simulation.grid.occupancy.size();
    int occCols = simulation.grid.occupancy[0].size();
    for (int y = 0; y < occRows; y++) {
        for (int x = 0; x < occCols; x++) {
            if (simulation.grid.occupancy[y][x] == 0) { // Wall
                glColor3f(0.0f, 0.0f, 0.0f);
                float left = x * cellSize;
                float top = y * cellSize;
                float right = left + cellSize;
                float bottom = top + cellSize;
                glBegin(GL_QUADS);
                glVertex2f(left, top);
                glVertex2f(right, top);
                glVertex2f(right, bottom);
                glVertex2f(left, bottom);
                glEnd();
            }
        }
    }
}

inline void renderVineRobot(const Simulation& simulation, float scaleFactor) {
    const VineRobot& vine = simulation.vr;
    if (!simulation.vrActive || vine.points.empty()) {
        return;
    }

    // Draw each vine point as a small green circle.
    glColor3f(0.0f, 0.8f, 0.0f); // Bright green.
    constexpr float pointRadius = 1.0f;  // radius in pixels.
    constexpr int circleSegments = 12;

    for (size_t i = 0; i < vine.points.size(); ++i) {
        float px = vine.points[i].first * scaleFactor;
        float py = vine.points[i].second * scaleFactor;
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(px, py);
        for (int j = 0; j <= circleSegments; ++j) {
            float angle = j * 2.0f * 3.14159265f / circleSegments;
            float cx = px + std::cos(angle) * pointRadius;
            float cy = py + std::sin(angle) * pointRadius;
            glVertex2f(cx, cy);
        }
        glEnd();
    }

    // Draw the tip with a darker green circle.
    auto tip = vine.points.back();
    float tipX = tip.first * scaleFactor;
    float tipY = tip.second * scaleFactor;
    glColor3f(0.0f, 0.5f, 0.0f); // Darker green.
    constexpr float tipRadius = 6.0f;
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(tipX, tipY);
    for (int i = 0; i <= circleSegments; ++i) {
        float angle = i * 2.0f * 3.14159265f / circleSegments;
        float cx = tipX + std::cos(angle) * tipRadius;
        float cy = tipY + std::sin(angle) * tipRadius;
        glVertex2f(cx, cy);
    }
    glEnd();

    // Draw the global turn line if it has been set.
    // We assume that if both endpoints are nonzero then the line should be drawn.
    //if (!(g_turnLine.start.first == 0.0f && g_turnLine.start.second == 0.0f &&
    //    g_turnLine.end.first == 0.0f && g_turnLine.end.second == 0.0f)) {
    //    glColor3f(1.0f, 0.0f, 0.0f); // Red for the turn line.
    //    glLineWidth(2.0f);
    //    glBegin(GL_LINES);
    //    glVertex2f(g_turnLine.start.first * scaleFactor, g_turnLine.start.second * scaleFactor);
    //    glVertex2f(g_turnLine.end.first * scaleFactor, g_turnLine.end.second * scaleFactor);
    //    glEnd();
    //}
}


inline void renderInterpolatedHeatMap(const Simulation& simulation, float scaleFactor) {
    int rows = simulation.grid.interpolatedHeat.size();
    if (rows == 0) return;
    int cols = simulation.grid.interpolatedHeat[0].size();
    float cellSize = simulation.grid.cellSize * scaleFactor;

    // Use the same fixed temperature range as in renderHeatMap.
    const float baseTemp = 20.0f;
    const float maxTemp = 37.0f;

    // Draw the interpolated heat map (blue-to-red scale).
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float temp = simulation.grid.interpolatedHeat[i][j];
            float norm = (temp - baseTemp) / (maxTemp - baseTemp);
            if (norm < 0.0f) norm = 0.0f;
            if (norm > 1.0f) norm = 1.0f;
            // Linear interpolation: light blue (0.8, 0.8, 1.0) at baseTemp to red (1.6, 0.0, 0.0) at maxTemp.
            float r = 0.8f + 0.8f * norm;
            float g = 0.8f - 0.8f * norm;
            float b = 1.0f - norm;
            glColor3f(r, g, b);

            float left = j * cellSize;
            float top = i * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;
            glBegin(GL_QUADS);
            glVertex2f(left, top);
            glVertex2f(right, top);
            glVertex2f(right, bottom);
            glVertex2f(left, bottom);
            glEnd();
        }
    }

    // Overlay walls (and holes, if desired) from the discovered occupancy grid.
    int occRows = simulation.grid.occupancy.size();
    int occCols = simulation.grid.occupancy[0].size();
    for (int y = 0; y < occRows; y++) {
        for (int x = 0; x < occCols; x++) {
            if (simulation.grid.occupancy[y][x] == 0) { // Wall
                glColor3f(0.0f, 0.0f, 0.0f);
            } else if (simulation.grid.occupancy[y][x] == 2) { // Optional: Hole overlay
                glColor3f(0.0f, 0.0f, 0.5f);
            } else {
                continue;
            }
            float left = x * cellSize;
            float top = y * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;
            glBegin(GL_QUADS);
            glVertex2f(left, top);
            glVertex2f(right, top);
            glVertex2f(right, bottom);
            glVertex2f(left, bottom);
            glEnd();
        }
    }
}

#pragma once
#include <simul.hpp>
#include <GLFW/glfw3.h>
#include <cmath>
#include <vector>
#include <vine_robot.hpp>

// ... existing rendering functions ...

// NEW: Render the gradient map (vector field) from the gradX and gradY grids.
inline void renderGradientMap(const Simulation& simulation, float scaleFactor) {
    int rows = simulation.known_grid.gradX.size();
    if (rows == 0) return;
    int cols = simulation.known_grid.gradX[0].size();
    float cellSize = simulation.known_grid.cellSize * scaleFactor;

    // Maximum magnitude when each component is capped to 1: sqrt(1^2 + 1^2)
    const float cappedMax = 0.1f;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // Get original gradients.
            float gx = simulation.known_grid.gradX[i][j];
            float gy = simulation.known_grid.gradY[i][j];

            // Cap each component to maximum absolute value of 1.
            if (std::fabs(gx) > 1.0f)
                gx = (gx > 0) ? 1.0f : -1.0f;
            if (std::fabs(gy) > 1.0f)
                gy = (gy > 0) ? 1.0f : -1.0f;

            float magnitude;
            if (!std::isfinite(gx) || !std::isfinite(gy)) {
                // Use the capped maximum if any component is non-finite.
                magnitude = cappedMax;
            }
            else {
                magnitude = std::sqrt(gx * gx + gy * gy);
            }

            // Normalize the magnitude to [0,1] using the capped maximum.
            float norm = magnitude / cappedMax;
            if (norm > 1.0f) norm = 1.0f;

            // Map the normalized value to a color gradient.
            // Here, flat areas (norm = 0) are light blue, steep areas (norm = 1) are red.
            float r = norm;             // Red increases with slope.
            float g = 1.0f - norm;        // Green decreases with slope.
            float b = 1.0f - norm * 0.5f; // Blue slightly decreases.
            glColor3f(r, g, b);

            float left = j * cellSize;
            float top = i * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;
            glBegin(GL_QUADS);
            glVertex2f(left, top);
            glVertex2f(right, top);
            glVertex2f(right, bottom);
            glVertex2f(left, bottom);
            glEnd();
        }
    }
}




inline void renderHeightMap(const Simulation& simulation, float scaleFactor, float fixedMin = -1, float fixedMax = 1) {
    int rows = simulation.known_grid.height.size();
    if (rows == 0) return;
    int cols = simulation.known_grid.height[0].size();
    float cellSize = simulation.known_grid.cellSize * scaleFactor;

    // Use fixed min and max values for normalization.
    float minHeight = fixedMin;
    float maxHeight = fixedMax;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float h = simulation.known_grid.height[i][j];
            if (!std::isfinite(h)) {
                // Render undefined (NaN) heights as black.
                glColor3f(0.0f, 0.0f, 0.0f);
            }
            else {
                // Normalize using the fixed range.
                float norm = (h - minHeight) / (maxHeight - minHeight);
                norm = std::clamp(norm, 0.0f, 1.0f);
                float r, g, b;
                // Use a "jet" colormap: blue -> cyan -> green -> yellow -> red.
                if (norm < 0.25f) {
                    r = 0.0f;
                    g = 4.0f * norm;
                    b = 1.0f;
                }
                else if (norm < 0.5f) {
                    r = 0.0f;
                    g = 1.0f;
                    b = 1.0f - 4.0f * (norm - 0.25f);
                }
                else if (norm < 0.75f) {
                    r = 4.0f * (norm - 0.5f);
                    g = 1.0f;
                    b = 0.0f;
                }
                else {
                    r = 1.0f;
                    g = 1.0f - 4.0f * (norm - 0.75f);
                    b = 0.0f;
                }
                glColor3f(r, g, b);
            }

            float left = j * cellSize;
            float top = i * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;

            glBegin(GL_QUADS);
            glVertex2f(left, top);
            glVertex2f(right, top);
            glVertex2f(right, bottom);
            glVertex2f(left, bottom);
            glEnd();
        }
    }
}



