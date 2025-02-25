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

      
    }

    // Parameters for bilinear interpolation:
    //   x0: left
    //   x1: right
    //   y0: top
    //   y1: bottom
    //   h00: Height at the top-left corner of the rectangle (at (x0, y0))
    //   h10: Height at the top-right corner of the rectangle (at (x1, y0))
    //   h01: Height at the bottom-left corner of the rectangle (at (x0, y1))
    //   h11: Height at the bottom-right corner of the rectangle (at (x1, y1))
    inline void interpolateHeightInRect(std::vector<std::vector<float>>& height, float x0, float y0, float x1, float y1, float h00, float h10, float h01, float h11, float cellSize)
    {
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


    // Helper: bilinear interpolation at (x,y) in the rectangle defined by (x0,y0) and (x1,y1)
    // with corner heights h00 (top-left), h10 (top-right), h01 (bottom-left), h11 (bottom-right)
    float bilinear(float x, float y,
        float x0, float y0, float x1, float y1,
        float h00, float h10, float h01, float h11)
    {
        float u = (x - x0) / (x1 - x0);
        float v = (y - y0) / (y1 - y0);
        return h00 * (1 - u) * (1 - v) +
            h10 * u * (1 - v) +
            h01 * (1 - u) * v +
            h11 * u * v;
    }

    // Interpolate within a rectangle with an extra inner constraint at (centerX, centerY) with height hCenter.
    void interpolateHeightInRectWithCenter(std::vector<std::vector<float>>& height,
        float x0, float y0, float x1, float y1,
        float h00, float h10, float h01, float h11,
        float centerX, float centerY, float hCenter,
        float cellSize)
    {
        // Determine grid indices corresponding to the physical rectangle.
        int colStart = std::max(0, static_cast<int>(std::floor(x0 / cellSize)));
        int colEnd = std::min(static_cast<int>(height[0].size()), static_cast<int>(std::ceil(x1 / cellSize)));
        int rowStart = std::max(0, static_cast<int>(std::floor(y0 / cellSize)));
        int rowEnd = std::min(static_cast<int>(height.size()), static_cast<int>(std::ceil(y1 / cellSize)));

        // Compute maximum distance from inner point to the corners
        float d0 = std::hypot(x0 - centerX, y0 - centerY);
        float d1 = std::hypot(x1 - centerX, y0 - centerY);
        float d2 = std::hypot(x0 - centerX, y1 - centerY);
        float d3 = std::hypot(x1 - centerX, y1 - centerY);
        float dMax = std::max({ d0, d1, d2, d3 });

        // Precompute the bilinear value at the inner point.
        float baseCenter = bilinear(centerX, centerY, x0, y0, x1, y1, h00, h10, h01, h11);

        // Iterate over each grid cell in the rectangle.
        for (int i = rowStart; i < rowEnd; ++i) {
            for (int j = colStart; j < colEnd; ++j) {
                // Compute cell center coordinates in meters.
                float cellCenterX = (j + 0.5f) * cellSize;
                float cellCenterY = (i + 0.5f) * cellSize;

                // Baseline bilinear interpolation.
                float fB = bilinear(cellCenterX, cellCenterY, x0, y0, x1, y1, h00, h10, h01, h11);

                // Compute distance from cell center to the specified inner point.
                float d = std::hypot(cellCenterX - centerX, cellCenterY - centerY);

                // Blending weight: full correction at the inner point (d=0), none at corners (d=dMax).
                float w = 1.0f - std::min(d / dMax, 1.0f);

                // Final interpolated height.
                height[i][j] = fB + w * (hCenter - baseCenter);
            }
        }
    }



    inline void addHole(std::vector<std::vector<float>>& height, const Hole& h, float cellSize)
    {
        int rows = height.size();
        if (rows == 0) return;
        int cols = height[0].size();

        if (h.shape == HoleShape::RECTANGLE) {
            int colStart = std::max(0, static_cast<int>(std::floor(h.topLeftX / cellSize)));
            int colEnd = std::min(cols, static_cast<int>(std::ceil((h.topLeftX + h.width) / cellSize)));
            int rowStart = std::max(0, static_cast<int>(std::floor(h.topLeftY / cellSize)));
            int rowEnd = std::min(rows, static_cast<int>(std::ceil((h.topLeftY + h.height) / cellSize)));
            for (int i = rowStart; i < rowEnd; i++) {
                for (int j = colStart; j < colEnd; j++) {
                    float cellCenterX = (j + 0.5f) * cellSize;
                    float cellCenterY = (i + 0.5f) * cellSize;
                    if (cellCenterX >= h.topLeftX && cellCenterX <= h.topLeftX + h.width &&
                        cellCenterY >= h.topLeftY && cellCenterY <= h.topLeftY + h.height) {
                        height[i][j] = -h.depth;
                    }
                }
            }
        }
        if (h.shape == HoleShape::CIRCLE) {
            // Convert the center from meters to grid coordinates.
            float centerCol_f = h.centerX / cellSize;
            float centerRow_f = h.centerY / cellSize;
            int centerCol = static_cast<int>(centerCol_f);
            int centerRow = static_cast<int>(centerRow_f);

            // Determine the radius in grid cells.
            float radius_cells = h.radius / cellSize;

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
                    float dx = cellCenterX - h.centerX;
                    float dy = cellCenterY - h.centerY;
                    float dist = std::sqrt(dx * dx + dy * dy);
                    if (dist <= h.radius) {
                        // Use a squared falloff: deeper near the center and shallower near the edge.
                        //float factor = 1.0f - (dist / radius) * (dist / radius) * (dist / radius);
                        float factor = 1.0f;
                        height[i][j] = -h.depth * factor;
                    }

                }
            }
        }
    }

    // Interpolate the height in a circular region.
    // All grid cells whose centers fall inside the circle (centerX, centerY, radius)
    // will receive an interpolated value that is hCenter at the circle’s center,
    // and hBoundary at the circle’s edge.
    void interpolateHeightInCircle(std::vector<std::vector<float>>& height,
        float centerX, float centerY, float radius,
        float hBoundary, float hCenter,
        float cellSize)
    {
        int rows = height.size();
        if (rows == 0) return;
        int cols = height[0].size();

        // Determine the grid bounding box that covers the circle.
        int colStart = std::max(0, static_cast<int>(std::floor((centerX - radius) / cellSize)));
        int colEnd = std::min(cols, static_cast<int>(std::ceil((centerX + radius) / cellSize)));
        int rowStart = std::max(0, static_cast<int>(std::floor((centerY - radius) / cellSize)));
        int rowEnd = std::min(rows, static_cast<int>(std::ceil((centerY + radius) / cellSize)));

        // Loop through the candidate grid cells.
        for (int i = rowStart; i < rowEnd; ++i) {
            for (int j = colStart; j < colEnd; ++j) {
                // Compute the cell center in meters.
                float cellCenterX = (j + 0.5f) * cellSize;
                float cellCenterY = (i + 0.5f) * cellSize;
                // Euclidean distance from the cell center to the circle center.
                float d = std::hypot(cellCenterX - centerX, cellCenterY - centerY);
                if (d <= radius) {
                    // Compute a weight: 1 at the center and 0 at the boundary.
                    float weight = 1.0f - (d / radius);
                    // Linearly blend the inner and boundary heights.
                    height[i][j] = hBoundary + (hCenter - hBoundary) * weight;
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

        interpolateHeightInRect(height, 10, 0, 14.5, 7, 0, 1, 0, 0, cellSize);
        interpolateHeightInRect(height, 14.5, 0, 20, 7, -0.5, 0, 0, 0, cellSize);
        interpolateHeightInRect(height, 6, 10, 11, 15, 0, 0, 0, 0.7, cellSize);
        interpolateHeightInRect(height, 6, 10, 11, 15, 0, 0, 0, 0.7, cellSize);
        //interpolateHeightInRectWithCenter(height, 0.5, 13, 5.5, 19, 0, 0, 0, 0, 3, 16, -0.7, cellSize);
        interpolateHeightInCircle(height, 3, 16, 3.5, 0, -0.7, cellSize);
        interpolateHeightInRect(height, 6, 15, 11, 20, 0, 0.7, 0, 0, cellSize);
        interpolateHeightInCircle(height, 16.5, 15, 2.5, 0, 1, cellSize);

        // Apply each hole.
        for (const Hole& h : holes)
            addHole(height, h, cellSize);

        //// Compute gradient maps.
        HeightMapGenerator::computeGradientMap(height, gradX, gradY, cellSize);
    }

} // namespace HeightMapGenerator
