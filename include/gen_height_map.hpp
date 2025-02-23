#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <queue>
#include <cmath>
#include <limits>

namespace HeightMapGenerator {

    // A constant to represent undefined height values.
    const float UNDEFINED_HEIGHT = std::numeric_limits<float>::quiet_NaN();

    // Existing interpolateHeightMap function…
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



    inline void generateHeightMap(std::vector<std::vector<float>>& height, std::vector<std::vector<float>> gradX, std::vector<std::vector<float>> gradY, float cellSize)
    {
        // Initialize the height map (for example, set a flat ground at 0)
        for (int i = 0; i < height.size(); i++) {
            for (int j = 0; j < height[0].size(); j++) {
                height[i][j] = 0.0f;
            }
        }
        // Introduce a discontinuity (e.g. a hole) at the center for testing:
        int centerRow = height.size() / 2;
        int centerCol = height[0].size() / 2;
        height[centerRow][centerCol] = std::numeric_limits<float>::infinity();

        // Compute the gradient components from the height map.
        HeightMapGenerator::computeGradientMap(height, gradX, gradY, cellSize);

    }
} // namespace HeightMapGenerator
