#pragma once
#include <simul.hpp>
#include <GLFW/glfw3.h>
#include <cmath>
#include <vector>
#include <vine_robot.hpp>

#include <algorithm>
#include <iomanip>

#include <fstream>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <string>

// Helper function to set the heat map color (same as in renderHeatMap())
inline void setHeatMapColor(float temp) {
    const float baseTemp = 20.0f;
    const float maxTemp = 37.0f;
    float norm = (temp - baseTemp) / (maxTemp - baseTemp);
    norm = std::clamp(norm, 0.0f, 1.0f);
    float t = norm;
    float r = (1.0f - t) * 0.980f + t * 0.290f;
    float g = (1.0f - t) * 0.941f + t * 0.000f;
    float b = (1.0f - t) * 1.000f + t * 0.118f;
    glColor3f(r, g, b);
}


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


inline void renderMeasurementGridAlpha(const Simulation& simulation, float scaleFactor) {
    int gridRows = simulation.grid.occupancy.size();
    int gridCols = simulation.grid.occupancy[0].size();
    float cellSize = simulation.known_grid.cellSize * scaleFactor;
    for (int y = 0; y < gridRows; ++y) {
        for (int x = 0; x < gridCols; ++x) {
            bool draw = false;
            int cell = simulation.grid.occupancy[y][x];
            // If cell is unexplored (-1), use grey.
            if (cell == -1) {
                glColor4f(0.0f, 0.0f, 0.0f, 0.5f); // grey
                draw = true;
            }
            else {
                // Otherwise, color as follows:
                // 0: wall -> black
                // 1: ground -> white
                // 2: hole -> dark blue
                switch (cell) {
                case 0:
                    //glColor3f(0.0f, 0.0f, 0.0f); // black
                    break;
                case 1:
                    //glColor4f(0.0f, 0.0f, 0.0f, 0.5f); // white
                    //draw = true;
                    break;
                case 2:
                    //glColor3f(0.0f, 0.0f, 0.5f); // dark blue
                    break;
                default:
                    //glColor3f(1.0f, 1.0f, 1.0f);
                    break;
                }
            }
            if (draw) {
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


inline void renderHeatMap(const Simulation& simulation, float scaleFactor) {
    int rows = simulation.known_grid.heat.size();
    if (rows == 0) return;
    int cols = simulation.known_grid.heat[0].size();
    float cellSize = simulation.known_grid.cellSize * scaleFactor;

    // Temperature range.
    const float baseTemp = 20.0f;  // Room temp: white (#faf0ff)
    const float maxTemp = 37.0f;   // Hottest: dark red (#4a001e)

    // For each cell in the heat grid.
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float temp = simulation.known_grid.heat[i][j];
            float norm = (temp - baseTemp) / (maxTemp - baseTemp);
            norm = std::clamp(norm, 0.0f, 1.0f);
            // Interpolate from white to dark red.
            // White (#faf0ff): (0.980, 0.941, 1.0)
            // Dark red (#4a001e): (0.290, 0.000, 0.118)
            float t = norm;
            float r = (1.0f - t) * 0.980f + t * 0.290f;
            float g = (1.0f - t) * 0.941f + t * 0.0f;
            float b = (1.0f - t) * 1.0f + t * 0.118f;
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
    const float holeColor[3] = { 0.0f, 44.0f / 255.0f, 45.0f / 255.0f };    // #002C2D
    // Overlay walls.
    int occRows = simulation.known_grid.occupancy.size();
    int occCols = simulation.known_grid.occupancy[0].size();
    for (int y = 0; y < occRows; y++) {
        for (int x = 0; x < occCols; x++) {
            int cell = simulation.known_grid.occupancy[y][x];
            if (cell == 0) { // Wall -> black
                glColor3f(0.0f, 0.0f, 0.0f);
            }
            else if (cell == 2) { // Hole -> blue (unchanged)
                glColor3f(holeColor[0], holeColor[1], holeColor[2]);
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

    // Draw person markers (unchanged).
    float cellSizePixels = simulation.known_grid.cellSize * scaleFactor;
    for (const auto& pos : simulation.personPositions) {
        // Center in the cell.
        float centerX = (pos.first / simulation.known_grid.cellSize + 0.5f) * cellSizePixels;
        float centerY = (pos.second / simulation.known_grid.cellSize + 0.5f) * cellSizePixels;
        float radiusScreen = 0.15f * scaleFactor;
        glColor3f(0.0f, 0.0f, 0.0f); // Black outline.
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
        float halfXSize = radiusScreen * 0.5f;
        glBegin(GL_LINES);
        glVertex2f(centerX - halfXSize, centerY - halfXSize);
        glVertex2f(centerX + halfXSize, centerY + halfXSize);
        glVertex2f(centerX - halfXSize, centerY + halfXSize);
        glVertex2f(centerX + halfXSize, centerY - halfXSize);
        glEnd();
    }
}


inline void renderHeatMapOnly(const Simulation& simulation, float scaleFactor) {
    int rows = simulation.known_grid.heat.size();
    if (rows == 0) return;
    int cols = simulation.known_grid.heat[0].size();
    float cellSize = simulation.known_grid.cellSize * scaleFactor;

    // Temperature range.
    const float baseTemp = 20.0f;  // Room temp: white (#faf0ff)
    const float maxTemp = 37.0f;   // Hottest: dark red (#4a001e)

    //// For each cell in the heat grid.
    //for (int i = 0; i < rows; i++) {
    //    for (int j = 0; j < cols; j++) {
    //        float temp = simulation.known_grid.heat[i][j];
    //        float norm = (temp - baseTemp) / (maxTemp - baseTemp);
    //        norm = std::clamp(norm, 0.0f, 1.0f);
    //        // Interpolate from white to dark red.
    //        // White (#faf0ff): (0.980, 0.941, 1.0)
    //        // Dark red (#4a001e): (0.290, 0.000, 0.118)
    //        float t = norm;
    //        float r = (1.0f - t) * 0.980f + t * 0.290f;
    //        float g = (1.0f - t) * 0.941f + t * 0.0f;
    //        float b = (1.0f - t) * 1.0f + t * 0.118f;
    //        if (r > 0.7 && g > 0.7 && b == 1)
    //            continue;
    //        glColor4f(r, g, b, 0.5);

    //        float left = j * cellSize;
    //        float top = i * cellSize;
    //        float right = left + cellSize;
    //        float bottom = top + cellSize;
    //        glBegin(GL_QUADS);
    //        glVertex2f(left, top);
    //        glVertex2f(right, top);
    //        glVertex2f(right, bottom);
    //        glVertex2f(left, bottom);
    //        glEnd();
    //    }
    //}
  
    // Draw person markers (unchanged).
    float cellSizePixels = simulation.known_grid.cellSize * scaleFactor;
    for (const auto& pos : simulation.personPositions) {
        // Center in the cell.
        float centerX = (pos.first / simulation.known_grid.cellSize + 0.5f) * cellSizePixels;
        float centerY = (pos.second / simulation.known_grid.cellSize + 0.5f) * cellSizePixels;
        float radiusScreen = 0.15f * scaleFactor;
        const int numSegments = 50;

        // Draw filled circle in red.
        glColor3f(1.0f, 0.0f, 0.0f); // Red fill.
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(centerX, centerY); // Center of circle.
        for (int i = 0; i <= numSegments; i++) {
            float angle = 2.0f * 3.14159265f * i / numSegments;
            float x = centerX + radiusScreen * std::cos(angle);
            float y = centerY + radiusScreen * std::sin(angle);
            glVertex2f(x, y);
        }
        glEnd();

        // Draw black outline.
        glColor3f(0.0f, 0.0f, 0.0f); // Black outline.
        glLineWidth(2.0f);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < numSegments; i++) {
            float angle = 2.0f * 3.14159265f * i / numSegments;
            float x = centerX + radiusScreen * std::cos(angle);
            float y = centerY + radiusScreen * std::sin(angle);
            glVertex2f(x, y);
        }
        glEnd();

        // Draw the cross lines.
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

    // Draw background: discovered ground cells use the heat map,
    // unknown cells are drawn in a lighter grey.
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (simulation.grid.occupancy[i][j] == -1) {
                glColor3f(0.8f, 0.8f, 0.8f);
            }
            else if (simulation.grid.occupancy[i][j] == 1) {
                float temp = simulation.grid.heat[i][j];
                if (temp == 0.0f)
                    glColor3f(1.0f, 1.0f, 1.0f);
                else
                    setHeatMapColor(temp);
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
    const float holeColor[3] = { 0.0f, 44.0f / 255.0f, 45.0f / 255.0f };    // #002C2D
    // Overlay walls and holes from the true occupancy grid.
    int occRows = simulation.known_grid.occupancy.size();
    int occCols = simulation.known_grid.occupancy[0].size();
    for (int y = 0; y < occRows; y++) {
        for (int x = 0; x < occCols; x++) {
            int trueCell = simulation.known_grid.occupancy[y][x];
            if (trueCell == 0 || trueCell == 2) {
                float left = x * cellSize;
                float top = y * cellSize;
                float right = left + cellSize;
                float bottom = top + cellSize;
                if (trueCell == 2) {
                    // For holes always fill with dark blue.
                    
                    glColor3f(holeColor[0], holeColor[1], holeColor[2]);
                    glBegin(GL_QUADS);
                    glVertex2f(left, top);
                    glVertex2f(right, top);
                    glVertex2f(right, bottom);
                    glVertex2f(left, bottom);
                    glEnd();
                }
                else {
                    // For walls (trueCell==0) use discovery status.
                    bool discovered = (simulation.grid.occupancy[y][x] == trueCell);
                    if (discovered) {
                        glColor3f(0.0f, 0.0f, 0.0f); // discovered wall: black
                        glBegin(GL_QUADS);
                        glVertex2f(left, top);
                        glVertex2f(right, top);
                        glVertex2f(right, bottom);
                        glVertex2f(left, bottom);
                        glEnd();
                    }
                    else {
                        glColor3f(0.3f, 0.3f, 0.3f); // undiscovered wall: dark grey outline
                        glLineWidth(1.0f);
                        float inset = cellSize * 0.2f;
                        glBegin(GL_LINE_LOOP);
                        glVertex2f(left + inset, top + inset);
                        glVertex2f(right - inset, top + inset);
                        glVertex2f(right - inset, bottom - inset);
                        glVertex2f(left + inset, bottom - inset);
                        glEnd();
                    }
                }
            }
        }
    }
}

inline void renderVineRobot(const Simulation& simulation, float scaleFactor) {
    const VineRobot& vine = simulation.vr;
    if (!simulation.vrActive || vine.points.empty()) {
        return;
    }

    glColor3f(0.0f, 0.8f, 0.0f); // Bright green.
    glLineWidth(2.0f); // Adjust line thickness if needed.
    glBegin(GL_LINE_STRIP);
    for (size_t i = 0; i < vine.points.size(); ++i) {
        float px = vine.points[i].first * scaleFactor;
        float py = vine.points[i].second * scaleFactor;
        glVertex2f(px, py);
    }
    glEnd();

    // Optionally, draw each vine point as a small green circle.
    constexpr float pointRadius = 1.0f;  // radius in pixels.
    constexpr int circleSegments = 12;
    for (size_t i = 0; i < vine.points.size(); ++i) {
        float px = vine.points[i].first * scaleFactor;
        float py = vine.points[i].second * scaleFactor;
        glColor3f(0.0f, 0.8f, 0.0f); // Maintain bright green.
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
    constexpr float tipRadius = 5.0f;
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(tipX, tipY);
    for (int i = 0; i <= circleSegments; ++i) {
        float angle = i * 2.0f * 3.14159265f / circleSegments;
        float cx = tipX + std::cos(angle) * tipRadius;
        float cy = tipY + std::sin(angle) * tipRadius;
        glVertex2f(cx, cy);
    }
    glEnd();
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

    // Define normalized breakpoints for the 7 stops.
    const float v0 = 0.0f;
    const float v1 = 0.1667f;
    const float v2 = 0.3333f;
    const float v3 = 0.5f;
    const float v4 = 0.6667f;
    const float v5 = 0.8333f;
    const float v6 = 1.0f;

    // Precomputed normalized RGB values for each stop.
    const float c0[3] = { 88.0f / 255.0f, 0.0f, 0.0f };          // #580000
    const float c1[3] = { 156.0f / 255.0f, 69.0f / 255.0f, 17.0f / 255.0f };  // #9C4511
    const float c2[3] = { 221.0f / 255.0f, 134.0f / 255.0f, 41.0f / 255.0f }; // #DD8629
    const float c3[3] = { 1.0f, 1.0f, 224.0f / 255.0f };          // #FFFFE0
    const float c4[3] = { 62.0f / 255.0f, 168.0f / 255.0f, 166.0f / 255.0f }; // #3EA8A6
    const float c5[3] = { 7.0f / 255.0f, 103.0f / 255.0f, 105.0f / 255.0f };  // #076769
    const float c6[3] = { 0.0f, 44.0f / 255.0f, 45.0f / 255.0f };    // #002C2D

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float h = simulation.known_grid.height[i][j];
            int cell = simulation.known_grid.occupancy[i][j];
            if (cell == 0) {
                glColor3f(0.0f, 0.0f, 0.0f);
            }
            else {
                if (!std::isfinite(h)) {
                    // Render undefined heights as black.
                    glColor3f(0.0f, 0.0f, 0.0f);
                }
                else {
                    // Normalize height using the fixed range.
                    float norm = (h - minHeight) / (maxHeight - minHeight);
                    norm = std::clamp(norm, 0.0f, 1.0f);
                    norm = 1.0f - norm;
                    float r, g, b;
                    if (norm < v1) {
                        float t = (norm - v0) / (v1 - v0);
                        r = c0[0] + t * (c1[0] - c0[0]);
                        g = c0[1] + t * (c1[1] - c0[1]);
                        b = c0[2] + t * (c1[2] - c0[2]);
                    }
                    else if (norm < v2) {
                        float t = (norm - v1) / (v2 - v1);
                        r = c1[0] + t * (c2[0] - c1[0]);
                        g = c1[1] + t * (c2[1] - c1[1]);
                        b = c1[2] + t * (c2[2] - c1[2]);
                    }
                    else if (norm < v3) {
                        float t = (norm - v2) / (v3 - v2);
                        r = c2[0] + t * (c3[0] - c2[0]);
                        g = c2[1] + t * (c3[1] - c2[1]);
                        b = c2[2] + t * (c3[2] - c2[2]);
                    }
                    else if (norm < v4) {
                        float t = (norm - v3) / (v4 - v3);
                        r = c3[0] + t * (c4[0] - c3[0]);
                        g = c3[1] + t * (c4[1] - c3[1]);
                        b = c3[2] + t * (c4[2] - c3[2]);
                    }
                    else if (norm < v5) {
                        float t = (norm - v4) / (v5 - v4);
                        r = c4[0] + t * (c5[0] - c4[0]);
                        g = c4[1] + t * (c5[1] - c4[1]);
                        b = c4[2] + t * (c5[2] - c4[2]);
                    }
                    else {
                        float t = (norm - v5) / (v6 - v5);
                        r = c5[0] + t * (c6[0] - c5[0]);
                        g = c5[1] + t * (c6[1] - c5[1]);
                        b = c5[2] + t * (c6[2] - c5[2]);
                    }
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
}

// Add this function to your simul_render.hpp

inline void renderFifthFigure(const Simulation& simulation, float scaleFactor) {
    int rows = simulation.known_grid.occupancy.size();
    int cols = simulation.known_grid.occupancy[0].size();
    float cellSize = simulation.known_grid.cellSize * scaleFactor;

    const float holeColor[3] = { 0.0f, 44.0f / 255.0f, 45.0f / 255.0f };    // #002C2D
    //glColor3f(holeColor[0], holeColor[1], holeColor[2]);

    // Draw background using discovered heat values or light grey if unknown.
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (simulation.grid.occupancy[i][j] != -1) {
                float temp = simulation.grid.heat[i][j];
                if (temp == 0.0f)
                    glColor3f(1.0f, 1.0f, 1.0f);
                else
                    setHeatMapColor(temp);
            }
            else {
                glColor3f(0.8f, 0.8f, 0.8f);  // lighter grey for unknown
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

    // Overlay walls/holes using the true occupancy grid.
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            int trueCell = simulation.known_grid.occupancy[i][j];
            if (trueCell == 0 || trueCell == 2) {
                float left = j * cellSize;
                float top = i * cellSize;
                float right = left + cellSize;
                float bottom = top + cellSize;
                if (trueCell == 2) {
                    // Always fill holes with dark blue.
                    glColor3f(holeColor[0], holeColor[1], holeColor[2]);
                    glBegin(GL_QUADS);
                    glVertex2f(left, top);
                    glVertex2f(right, top);
                    glVertex2f(right, bottom);
                    glVertex2f(left, bottom);
                    glEnd();
                }
                else {
                    // For walls (trueCell==0), check discovery status.
                    bool discovered = (simulation.grid.occupancy[i][j] == trueCell);
                    if (discovered) {
                        glColor3f(0.0f, 0.0f, 0.0f); // discovered wall: black
                        glBegin(GL_QUADS);
                        glVertex2f(left, top);
                        glVertex2f(right, top);
                        glVertex2f(right, bottom);
                        glVertex2f(left, bottom);
                        glEnd();
                    }
                    else {
                        // Undiscovered wall: draw a thin outline.
                        glColor3f(0.3f, 0.3f, 0.3f);
                        glLineWidth(1.0f);
                        float inset = cellSize * 0.2f;
                        glBegin(GL_LINE_LOOP);
                        glVertex2f(left + inset, top + inset);
                        glVertex2f(right - inset, top + inset);
                        glVertex2f(right - inset, bottom - inset);
                        glVertex2f(left + inset, bottom - inset);
                        glEnd();
                    }
                }
            }
        }
    }

    // Overlay the robots.
    renderRobots(simulation, scaleFactor);
    renderVineRobot(simulation, scaleFactor);

    // Draw people as filled red circles.
    // Use the same cell size conversion for positioning.
    float cellSizePixels = simulation.known_grid.cellSize * scaleFactor;
    const int numSegments = 50;
    for (const auto& pos : simulation.personPositions) {
        // Compute the cell center where the person is located.
        float centerX = (pos.first / simulation.known_grid.cellSize + 0.5f) * cellSizePixels;
        float centerY = (pos.second / simulation.known_grid.cellSize + 0.5f) * cellSizePixels;
        // Use the standard person radius.
        float radiusScreen = 0.15f * scaleFactor;
        glColor3f(0.290, 0.000, 0.118); // filled red
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(centerX, centerY);  // center
        for (int k = 0; k <= numSegments; k++) {
            float angle = 2.0f * 3.14159265f * k / numSegments;
            float x = centerX + radiusScreen * cosf(angle);
            float y = centerY + radiusScreen * sinf(angle);
            glVertex2f(x, y);
        }
        glEnd();
    }
}

// saving data as csv

void saveHeightMapToCSV(const Simulation& simulation, const std::string& filename) {
    int rows = simulation.known_grid.height.size();
    if (rows == 0) return;
    int cols = simulation.known_grid.height[0].size();
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            ofs << simulation.known_grid.height[i][j];
            if (j < cols - 1)
                ofs << ",";
        }
        ofs << "\n";
    }
    ofs.close();
}
void saveOccupancyToCSV(const Simulation& simulation, const std::string& filename) {
    int rows = simulation.known_grid.occupancy.size();
    if (rows == 0) return;
    int cols = simulation.known_grid.occupancy[0].size();
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            ofs << simulation.known_grid.occupancy[i][j];
            if (j < cols - 1)
                ofs << ",";
        }
        ofs << "\n";
    }
    ofs.close();
}
void saveHeatMapToCSV(const Simulation& simulation, const std::string& filename) {
    int rows = simulation.known_grid.heat.size();
    if (rows == 0) return;
    int cols = simulation.known_grid.heat[0].size();

    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            ofs << simulation.known_grid.heat[i][j];
            if (j < cols - 1)
                ofs << ",";
        }
        ofs << "\n";
    }

    ofs.close();
}
