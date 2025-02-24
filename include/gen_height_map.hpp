#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <queue>
#include <cmath>
#include <limits>
#include <office_elements.hpp>

namespace HeightMapGenerator {

    // A constant to represent undefined height values.
    const float UNDEFINED_HEIGHT = std::numeric_limits<float>::quiet_NaN();

    // Existing interpolateHeightMap function�
    inline void interpolateHeightMap(const std::vector<std::vector<float>>& height,
        std::vector<std::vector<float>>& interpolated,
        float cellSize) {
        int rows = height.size();
        if (rows == 0) return;
        int cols = height[0].size();
        struct MeasuredPoint { int i; int j; float value; };
        std::vector<MeasuredPoint> measured;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (!std::isnan(height[i][j])) {
                    measured.push_back({ i, j, height[i][j] });
                }
            }
        }
        if (measured.empty()) {
            interpolated = height;
            return;
        }
        interpolated.resize(rows, std::vector<float>(cols, 0.0f));
        const float p = 2.0f;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (!std::isnan(height[i][j])) {
                    interpolated[i][j] = height[i][j];
                }
                else {
                    float numerator = 0.0f, denominator = 0.0f;
                    for (const auto& m : measured) {
                        float d = std::sqrt((i - m.i) * (i - m.i) + (j - m.j) * (j - m.j));
                        if (d < 1e-4f) { numerator = m.value; denominator = 1.0f; break; }
                        float weight = 1.0f / std::pow(d, p);
                        numerator += weight * m.value;
                        denominator += weight;
                    }
                    interpolated[i][j] = (denominator != 0.0f) ? (numerator / denominator) : 0.0f;
                }
            }
        }
    }

    // New: Compute the gradient components (gradX and gradY) from the height map.
    inline void computeGradientMap(const std::vector<std::vector<float>>& height,
        std::vector<std::vector<float>>& gradX,
        std::vector<std::vector<float>>& gradY,
        float cellSize) {
        int rows = height.size();
        if (rows < 3) return;
        int cols = height[0].size();
        gradX.resize(rows, std::vector<float>(cols, 0.0f));
        gradY.resize(rows, std::vector<float>(cols, 0.0f));
        for (int i = 1; i < rows - 1; i++) {
            for (int j = 1; j < cols - 1; j++) {
                if (std::isnan(height[i][j]) || std::isnan(height[i - 1][j]) ||
                    std::isnan(height[i + 1][j]) || std::isnan(height[i][j - 1]) ||
                    std::isnan(height[i][j + 1])) {
                    gradX[i][j] = std::numeric_limits<float>::infinity();
                    gradY[i][j] = std::numeric_limits<float>::infinity();
                }
                else {
                    gradX[i][j] = (height[i][j + 1] - height[i][j - 1]) / (2.0f * cellSize);
                    gradY[i][j] = (height[i + 1][j] - height[i - 1][j]) / (2.0f * cellSize);
                }
            }
        }
        // For simplicity, copy interior values to the borders.
        for (int i = 0; i < rows; i++) {
            gradX[i][0] = (cols > 1) ? gradX[i][1] : 0.0f;
            gradX[i][cols - 1] = (cols > 1) ? gradX[i][cols - 2] : 0.0f;
        }
        for (int j = 0; j < cols; j++) {
            gradY[0][j] = (rows > 1) ? gradY[1][j] : 0.0f;
            gradY[rows - 1][j] = (rows > 1) ? gradY[rows - 2][j] : 0.0f;
        }
    }

    inline void addHole(std::vector<std::vector<float>>& height,
        float centerX, float centerY,
        float radius, float depth,
        float cellSize)
    {
        int rows = height.size();
        if (rows == 0) return;
        int cols = height[0].size();

        // Convert the center from meters to grid coordinates.
        float centerCol_f = centerX / cellSize;
        float centerRow_f = centerY / cellSize;
        int centerCol = static_cast<int>(centerCol_f);
        int centerRow = static_cast<int>(centerRow_f);

        // Determine the radius in grid cells.
        float radius_cells = radius / cellSize;

        // Determine the bounding box in grid indices.
        int rowStart = std::max(0, centerRow - static_cast<int>(radius_cells));
        int rowEnd = std::min(rows, centerRow + static_cast<int>(radius_cells) + 1);
        int colStart = std::max(0, centerCol - static_cast<int>(radius_cells));
        int colEnd = std::min(cols, centerCol + static_cast<int>(radius_cells) + 1);

        for (int i = rowStart; i < rowEnd; i++) {
            for (int j = colStart; j < colEnd; j++) {
                // Compute the cell's center coordinates in meters.
                float cellCenterX = (j + 0.5f) * cellSize;
                float cellCenterY = (i + 0.5f) * cellSize;
                // Compute the distance from the cell center to the hole center.
                float dx = cellCenterX - centerX;
                float dy = cellCenterY - centerY;
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist <= radius) {
                    // Use a squared falloff: deeper near the center and shallower near the edge.
                    //float factor = 1.0f - (dist / radius) * (dist / radius) * (dist / radius);
                    float factor = 1.0f;
                    height[i][j] = -depth * factor;
                }

            }
        }
    }

    // Parameters for bilinear interpolation:
    //   h00: Height at the bottom-left corner of the rectangle (at (x0, y0))
    //   h10: Height at the bottom-right corner of the rectangle (at (x1, y0))
    //   h01: Height at the top-left corner of the rectangle (at (x0, y1))
    //   h11: Height at the top-right corner of the rectangle (at (x1, y1))
    inline void interpolateHeightInRect(std::vector<std::vector<float>>& height, float x0, float y0, float x1, float y1, float h00, float h01, float h10, float h11, float cellSize) {
        int rows = height.size();
        if (rows == 0) return;
        int cols = height[0].size();

        // Convert the physical rectangle bounds (meters) to grid indices.
        int colStart = std::max(0, static_cast<int>(std::floor(x0 / cellSize)));
        int colEnd = std::min(cols, static_cast<int>(std::ceil(x1 / cellSize)));
        int rowStart = std::max(0, static_cast<int>(std::floor(y0 / cellSize)));
        int rowEnd = std::min(rows, static_cast<int>(std::ceil(y1 / cellSize)));

        // Compute the width and height of the rectangle in meters.
        float width_m = x1 - x0;
        float height_m = y1 - y0;

        // Iterate over all cells in the rectangular region.
        for (int i = rowStart; i < rowEnd; i++) {
            for (int j = colStart; j < colEnd; j++) {
                // Compute the cell center in meters.
                float cellCenterX = (j + 0.5f) * cellSize;
                float cellCenterY = (i + 0.5f) * cellSize;
                // Clamp cell center coordinates to the rectangle, in case of rounding.
                cellCenterX = std::max(x0, std::min(cellCenterX, x1));
                cellCenterY = std::max(y0, std::min(cellCenterY, y1));
                // Compute normalized coordinates u and v in [0,1].
                float u = (cellCenterX - x0) / width_m;
                float v = (cellCenterY - y0) / height_m;
                // Bilinear interpolation:
                float hVal = h00 * (1.0f - u) * (1.0f - v)
                    + h10 * u * (1.0f - v)
                    + h01 * (1.0f - u) * v
                    + h11 * u * v;
                height[i][j] = hVal;
            }
        }
    }

    inline void addHoleRect(std::vector<std::vector<float>>& height,
        float topLeftX, float topLeftY,
        float width, float heightRect,
        float depth, float cellSize)
    {
        int rows = height.size();
        if (rows == 0) return;
        int cols = height[0].size();

        int colStart = std::max(0, static_cast<int>(std::floor(topLeftX / cellSize)));
        int colEnd = std::min(cols, static_cast<int>(std::ceil((topLeftX + width) / cellSize)));
        int rowStart = std::max(0, static_cast<int>(std::floor(topLeftY / cellSize)));
        int rowEnd = std::min(rows, static_cast<int>(std::ceil((topLeftY + heightRect) / cellSize)));

        for (int i = rowStart; i < rowEnd; i++) {
            for (int j = colStart; j < colEnd; j++) {
                float cellCenterX = (j + 0.5f) * cellSize;
                float cellCenterY = (i + 0.5f) * cellSize;
                if (cellCenterX >= topLeftX && cellCenterX <= topLeftX + width &&
                    cellCenterY >= topLeftY && cellCenterY <= topLeftY + heightRect) {
                    height[i][j] = -depth;
                }
            }
        }
    }

    inline void generateHeightMap(std::vector<std::vector<float>>& height,
        std::vector<std::vector<float>>& gradX,
        std::vector<std::vector<float>>& gradY,
        float cellSize,
        const std::vector<Hole>& holes)
    {
        int rows = height.size();
        if (rows == 0) return;
        int cols = height[0].size();

        // Initialize the height map to baseline (0.0 m)
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                height[i][j] = 0.0f;

        // (Optional: perform other interpolation here.)

        // Apply each hole.
        for (const Hole& h : holes) {
            if (h.shape == HoleShape::CIRCLE)
                addHole(height, h.centerX, h.centerY, h.radius, h.depth, cellSize);
            else if (h.shape == HoleShape::RECTANGLE)
                addHoleRect(height, h.topLeftX, h.topLeftY, h.width, h.height, h.depth, cellSize);
        }

        // Compute gradient maps.
        HeightMapGenerator::computeGradientMap(height, gradX, gradY, cellSize);
    }

} // namespace HeightMapGenerator
