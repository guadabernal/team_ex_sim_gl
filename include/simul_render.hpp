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
        // Convert world coordinates (meters) to screen coordinates.
        float sx = robot.x * scaleFactor;
        float sy = robot.y * scaleFactor;

        // Use the robot's own size (in meters).
        float size = robot.size;  // e.g., 0.15m

        // Define triangle vertices in the robot's local coordinate system.
        // The triangle points in the +x direction.
        float tipX = size;      // Tip (front)
        float tipY = 0.0f;
        float baseLeftX = -size;     // Rear left
        float baseLeftY = -size / 2.0f;
        float baseRightX = -size;     // Rear right
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

        // Set the color for the robot (yellow).
        glColor3f(1.0f, 1.0f, 0.0f);
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
            case 0: // Wall (red)
                glColor3f(1.0f, 0.0f, 0.0f);
                break;
            case 1: // Ground (green)
                glColor3f(0.0f, 0.8f, 0.0f);
                break;
            case 2: // Hole (blue)
                glColor3f(0.0f, 0.0f, 0.8f);
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