#pragma once
#include <simul.hpp>
#include <GLFW/glfw3.h>

// -----------------------------------------------------------------------------
// Function: renderRobots
// Renders each robot in the simulation as a small triangle whose tip points in
// the direction of its theta. The robot positions (in world coordinates) are
// converted to screen coordinates using the provided scaleFactor.
// -----------------------------------------------------------------------------
inline void renderRobots(const Simulation& simulation, float scaleFactor) {
    for (const auto& robot : simulation.rr) {
        if (robot.dead && robot.battery > 0)
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

// A helper function to render the occupancy grid.
inline void renderGrid(const Simulation& simulation, float scaleFactor) {
    int gridRows = simulation.known_grid.occupancy.size();
    int gridCols = simulation.known_grid.occupancy[0].size();
    float cellSize = simulation.known_grid.scale_m * scaleFactor;
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
    float cellSize = simulation.known_grid.scale_m * scaleFactor;
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

// Renders the heat map using known_grid.heat.
// Cells at baseTemp (20 deg C) or below are white.
// For cells above baseTemp, the color becomes red with increasing intensity.
// A sensitivity factor is used so that even small temperature differences become noticeable.
// Renders the heat map using known_grid.heat.
// Cells at or below the base temperature (20 deg C) are white.
// For cells above baseTemp, the color becomes increasingly red,
// except that cells at the person temperature (37 deg C) are drawn in black.
// After drawing the heat background, wall cells (occupancy==0) are overlaid in black,
// then the robots are rendered on top.
inline void renderHeatMap(const Simulation& simulation, float scaleFactor) {
    int rows = simulation.known_grid.heat.size();
    if (rows == 0) return;
    int cols = simulation.known_grid.heat[0].size();
    float cellSize = simulation.known_grid.scale_m * scaleFactor;

    const float baseTemp = 20.0f;
    const float personTemp = 37.0f;
    const float sensitivity = 5.0f; // degrees above base for full red intensity

    // Draw the heat map background.
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float temp = simulation.known_grid.heat[i][j];

            // If the cell is at the person's temperature, color it black.
            if (std::fabs(temp - personTemp) < 0.01f) {
                glColor3f(0.0f, 0.0f, 0.0f);
            }
            else if (temp <= baseTemp) {
                glColor3f(1.0f, 1.0f, 1.0f);
            }
            else {
                float diff = temp - baseTemp;
                float norm = diff / sensitivity;
                if (norm > 1.0f)
                    norm = 1.0f;
                // At norm==0, color is white; at norm==1, color is fully red.
                glColor3f(1.0f, 1.0f - norm, 1.0f - norm);
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

    // Overlay walls from the occupancy grid.
    int occRows = simulation.known_grid.occupancy.size();
    int occCols = simulation.known_grid.occupancy[0].size();
    for (int y = 0; y < occRows; y++) {
        for (int x = 0; x < occCols; x++) {
            if (simulation.known_grid.occupancy[y][x] == 0) { // Wall
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

    // Render the robots on top.
    renderRobots(simulation, scaleFactor);
}

inline void renderDiscoveredHeatMap(const Simulation& simulation, float scaleFactor) {
    int rows = simulation.grid.heat.size();
    if (rows == 0) return;
    int cols = simulation.grid.heat[0].size();
    float cellSize = simulation.known_grid.scale_m * scaleFactor;

    const float baseTemp = 20.0f;
    const float personTemp = 37.0f;
    const float sensitivity = 5.0f; // degrees above base for full red intensity

    // Draw the discovered heat map background.
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float temp = simulation.grid.heat[i][j];
            if (std::fabs(temp - personTemp) < 0.01f) {
                glColor3f(0.0f, 0.0f, 0.0f);
            }
            else if (temp <= baseTemp) {
                glColor3f(1.0f, 1.0f, 1.0f);
            }
            else {
                float diff = temp - baseTemp;
                float norm = diff / sensitivity;
                if (norm > 1.0f)
                    norm = 1.0f;
                glColor3f(1.0f, 1.0f - norm, 1.0f - norm);
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

    // Optionally overlay discovered walls.
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


inline void renderGridWithDiscovery(const Simulation& simulation, float scaleFactor) {
    // For the measurement display, we only need the discovered grid rendered
    // with the new colors (unexplored = grey, walls and holes as before).
    renderMeasurementGrid(simulation, scaleFactor);
}