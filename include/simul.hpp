#pragma once
#include <vector>
#include <utility>
#include <cmath>
#include <tuple>
#include <vine_robot.hpp>
#include <rescue_robot.hpp>
#include <gen_occupancy.hpp>
#include <gen_height_map.hpp>

struct Grid {
    float cellSize;  // Size of each cell in meters
    int rows = 0;
    int cols = 0;
    std::vector<std::vector<float>> co2;
    std::vector<std::vector<float>> heat;
    std::vector<std::vector<int>> ground_type;
    std::vector<std::vector<float>> interpolatedHeat;
    std::vector<std::vector<float>> height;
    std::vector<std::vector<float>> gradX;  
    std::vector<std::vector<float>> gradY;  
    std::vector<std::vector<int>> occupancy;
    std::vector<std::vector<int>> foundBy;

    // Constructor: now initializes gradX and gradY as well.
    Grid(int rows, int cols, float cellSize)
        : 
        rows(rows), cols(cols),
        co2(rows, std::vector<float>(cols, 0.0f)),
        heat(rows, std::vector<float>(cols, -1)),
        interpolatedHeat(rows, std::vector<float>(cols, 0.0f)),
        ground_type(rows, std::vector<int>(cols, 0)),
        height(rows, std::vector<float>(cols, 0.0f)),
        gradX(rows, std::vector<float>(cols, 0.0f)),  // NEW
        gradY(rows, std::vector<float>(cols, 0.0f)),  // NEW
        occupancy(rows, std::vector<int>(cols, -1)),
        foundBy(rows, std::vector<int>(cols, -1)),
        cellSize(cellSize)
    {}
};

inline void updateInterpolatedHeatMap(Grid& grid) {
    int rows = grid.heat.size();
    if (rows == 0) return;
    int cols = grid.heat[0].size();

    
    // Collect all measured points (i, j, value) from grid.heat.
    std::vector<std::tuple<int, int, float>> measured;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (grid.heat[i][j] != -1) {
                measured.push_back(std::make_tuple(i, j, grid.heat[i][j]));
            }
        }
    }
    // If no measured points exist, just copy the heat grid.
    if (measured.empty()) {
        grid.interpolatedHeat = grid.heat;
        return;
    }

    // Use inverse distance weighting to interpolate every cell.
    // p is the power parameter (commonly 2).
    float p = 2.0f;
    // Resize (or reinitialize) the interpolatedHeat grid.
    grid.interpolatedHeat.resize(rows, std::vector<float>(cols, 0.0f));
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // If a cell is measured, copy its value directly.
            if (grid.heat[i][j] != -1) {
                grid.interpolatedHeat[i][j] = grid.heat[i][j];
            }
            else {
                float numerator = 0.0f;
                float denominator = 0.0f;
                // Loop through all measured points.
                for (const auto& tup : measured) {
                    int mi, mj;
                    float val;
                    std::tie(mi, mj, val) = tup;
                    // Compute Euclidean distance in grid units.
                    float d = (i - mi) * (i - mi) + (j - mj) * (j - mj);
                    // Avoid division by zero: if the distance is extremely small, use the measured value.
                    if (d < 0.0001f) {
                        numerator = val;
                        denominator = 1.0f;
                        break;
                    }
                    float weight = 1.0f / d;
                    numerator += weight * val;
                    denominator += weight;
                }
                grid.interpolatedHeat[i][j] = (denominator != 0.0f) ? (numerator / denominator) : 0.0f;
            }
        }
    }
    // Force a measured value at physical (5,5).
    // Assuming grid.cellSize is the cell size (e.g., 0.05m), then:
    int centerIdx = static_cast<int>(5.0f / grid.cellSize);
    if (centerIdx < rows && centerIdx < cols) {
        grid.interpolatedHeat[centerIdx][centerIdx] = 500.0f;
    }

}
inline void heatmapUpdate(const std::vector<std::vector<int>>& occupancy,std::vector<std::vector<float>>& heat,float dt,float dx)
{
    int rows = heat.size();
    if (rows == 0) return;
    int cols = heat[0].size();
    std::vector<std::vector<float>> newHeat = heat;

    // Conduction parameters.
    const float D_air = 2e-5f;   // m^2/s for open air.
    const float D_wall = 1e-11f; // For walls.

    // First: update via conduction (as before).
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            int occ = occupancy[i][j];
            float D = (occ == 0) ? D_wall : D_air;
            float neighborSum = 0.0f;
            int count = 0;
            if (i > 0) { neighborSum += heat[i - 1][j]; count++; }
            if (i < rows - 1) { neighborSum += heat[i + 1][j]; count++; }
            if (j > 0) { neighborSum += heat[i][j - 1]; count++; }
            if (j < cols - 1) { neighborSum += heat[i][j + 1]; count++; }
            float laplacian = neighborSum - count * heat[i][j];
            newHeat[i][j] = heat[i][j] + D * dt / (dx * dx) * laplacian;
            if (newHeat[i][j] < 0)
                newHeat[i][j] = 0;
        }
    }

    const float convectionFactor = 0.1f;       // Weight for open-air neighbors.
    const float wallConvectionFactor = 0.02f;    // Lower weight for wall neighbors.
    std::vector<std::vector<float>> mixedHeat = newHeat;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // Start with the current cell's temperature (weight 1).
            float weightedSum = newHeat[i][j];
            float weightTotal = 1.0f;

            // Up neighbor.
            if (i > 0) {
                float weight = (occupancy[i - 1][j] == 0) ? wallConvectionFactor : convectionFactor;
                weightedSum += newHeat[i - 1][j] * weight;
                weightTotal += weight;
            }
            // Down neighbor.
            if (i < rows - 1) {
                float weight = (occupancy[i + 1][j] == 0) ? wallConvectionFactor : convectionFactor;
                weightedSum += newHeat[i + 1][j] * weight;
                weightTotal += weight;
            }
            // Left neighbor.
            if (j > 0) {
                float weight = (occupancy[i][j - 1] == 0) ? wallConvectionFactor : convectionFactor;
                weightedSum += newHeat[i][j - 1] * weight;
                weightTotal += weight;
            }
            // Right neighbor.
            if (j < cols - 1) {
                float weight = (occupancy[i][j + 1] == 0) ? wallConvectionFactor : convectionFactor;
                weightedSum += newHeat[i][j + 1] * weight;
                weightTotal += weight;
            }

            // Compute the weighted average.
            float neighborAvg = weightedSum / weightTotal;
            mixedHeat[i][j] = neighborAvg;
        }
    }
    heat = mixedHeat;
}
inline void injectHeatSources(std::vector<std::vector<float>>& heat, const std::vector<std::pair<float, float>>& sourcePositions, float personTemp, float scale)
{
    int rows = heat.size();
    if (rows == 0) return;
    int cols = heat[0].size();

    // Person diameter is 30cm; therefore the radius is 15cm (0.15m).
    float personRadius = 0.15f; // in meters
    // Compute how many cells correspond to the person radius.
    int radiusCells = static_cast<int>(std::ceil(personRadius / scale));

    // For each heat source...
    for (int i = 0; i < sourcePositions.size(); i++) {
        // The center of the person is given in grid coordinates.
        int centerX = static_cast<int>(sourcePositions[i].first /scale);
        int centerY = static_cast<int>(sourcePositions[i].second / scale);

        // Loop over a bounding square around the center.
        for (int y = centerY - radiusCells; y <= centerY + radiusCells; y++) {
            for (int x = centerX - radiusCells; x <= centerX + radiusCells; x++) {
                // Ensure (x,y) is within bounds.
                if (y < 0 || y >= rows || x < 0 || x >= cols)
                    continue;
                // Convert the grid cell offset to meters.
                float dx = (x - centerX) * scale;
                float dy = (y - centerY) * scale;
                // If the cell is within the circular area...
                if (std::sqrt(dx * dx + dy * dy) <= personRadius) {
                    heat[y][x] = personTemp;
                }
            }
        }
    }
}

struct SimConsts
{
    float cellSize = 0.05f;    // meters
    float totalSize = 10;      // meters
    int nRobots = 10;          // number of robots
    float muHoleSize = 0.5;
    float sigmaHoleSize = 0.2;
    int nHoles = 0;
    int nPeople = 0;
    float maxTime = 30;      // simulation maximum time
    float dt = 0.01;
    float rrEnable = false;
    float vrEnable = false;
    float vrX = -1;
    float vrY = -1;
    float vrAngle = -1;
    float rrX = -1;
    float rrY = -1;
    float rrAngle = -1;
    float rrTime = -1;
    int getGridCols() const { return static_cast<int>(totalSize / cellSize); }
    int getGridRows() const { return static_cast<int>(totalSize / cellSize); }
};

class Simulation {
public:
    SimConsts consts;
    Grid known_grid;
    Grid grid;

    std::vector<std::pair<float, float>> coverageHistory;


    std::vector<RescueRobot> rr;
    VineRobot vr;

    bool rrActive;
    bool vrActive;

    float t = 0;
    std::vector<std::pair<float, float>> personPositions;
    std::vector<float> personFoundTimes;
    std::vector<Hole> holes;

    int nextRrSpawnIndex;
    int updateCounter = 0;
    bool lastVineReversingState;
    float delta_drop_time;

    Simulation(const SimConsts& s)
        : known_grid(s.getGridRows(), s.getGridCols(), s.cellSize)
        , grid(s.getGridRows(), s.getGridRows(), s.cellSize)
        , consts(s)
        , rrActive(false)
        , vrActive(false)
        , nextRrSpawnIndex(0)
        , lastVineReversingState(false)
    {
        initializeHeatMap(10.0f, 20.0f);
        generateOfficeMap(known_grid.occupancy, consts.cellSize, 0.15f, 0.9f);
        holes = generateHolesList_custom1();
        updateOccupancyWithHoles(known_grid.occupancy, holes, consts.cellSize);
        HeightMapGenerator::generateHeightMap(known_grid.height, known_grid.gradX, known_grid.gradY, consts.cellSize, holes);

        personPositions.push_back({ 1.5f, 1.5f });
        personPositions.push_back({ 7.0f, 5.8f });
        personPositions.push_back({ 19.0f, 6.5f });
        personPositions.push_back({ 12.0f, 15.3f });

        personFoundTimes.resize(personPositions.size(), -1.0f);
    }

    void initializeRescueRobots(float startX, float startY, float startTheta, std::mt19937* rng_ptr) {
        rr.clear();
        delta_drop_time = consts.rrTime;
        for (int i = 0; i < consts.nRobots; i++) {
            RescueRobot robot(startX, startY, startTheta, consts.rrTime * i, consts.cellSize, true, true, rng_ptr);
            rr.push_back(robot);
        }
        nextRrSpawnIndex = 0;
    }

    void initializeVineRobot(float startX, float startY, float startTheta, std::mt19937* rng_ptr) {
        vr = VineRobot(startX, startY, startTheta, rng_ptr);
    }

    void HeatUpdate_forPlot() {
        heatmapUpdate(known_grid.occupancy, known_grid.heat, consts.dt, consts.cellSize);
        injectHeatSources(known_grid.heat, personPositions, 37.0f, consts.cellSize);
    }

    bool update() {
        heatmapUpdate(known_grid.occupancy, known_grid.heat, consts.dt, consts.cellSize);
        injectHeatSources(known_grid.heat, personPositions, 37.0f, consts.cellSize);

        for (size_t i = 0; i < personPositions.size(); i++) {
            if (personFoundTimes[i] < 0) {
                for (const auto& robot : rr) {
                    float dx = robot.x - personPositions[i].first;
                    float dy = robot.y - personPositions[i].second;
                    if (std::sqrt(dx * dx + dy * dy) < 0.50f) {
                        personFoundTimes[i] = t;
                        std::cout << "Rescue Robot " << robot.id << " found person " << i << " at time " << t << " sec." << std::endl;
                        break;
                    }
                }
            }
        }

        if (vrActive) {
            vr.move(known_grid.occupancy, known_grid.cellSize, consts.dt);
            if (!rrActive) {
                vr.measure(known_grid.occupancy, grid.occupancy, grid.foundBy, known_grid.heat, grid.heat, known_grid.cellSize, t);
            }
        }

        if (rrActive && nextRrSpawnIndex < rr.size() && t >= rr[nextRrSpawnIndex].spawnTime) {
            std::pair<float, float> spawnPoint;
            bool dropNow = false;
            if (vrActive) {
                // If vine is active, drop off only at the moment when it starts reversing.
                if (delta_drop_time == 0) {
                    if (!lastVineReversingState && vr.reversing) {
                        spawnPoint = vr.tip();
                        dropNow = true;
                    }
                }
                else
                {
                    spawnPoint = vr.tip();
                    dropNow = true;
                }
                
            }
            else {
                // If vine is off, drop off immediately at the rescue robot's starting location.
                // (Assuming that in createSimulation, we set rrX and rrY to the vine's start if vine is off.)
                spawnPoint = { consts.rrX, consts.rrY };
                dropNow = true;
            }

            if (dropNow) {
                float requiredDistance = 1.2f * rr[nextRrSpawnIndex].size;
                float scale = known_grid.cellSize;
                int gridRows = known_grid.occupancy.size();
                int gridCols = known_grid.occupancy[0].size();

                int startRow = std::max(0, (int)std::floor((spawnPoint.second - requiredDistance) / scale));
                int endRow = std::min(gridRows - 1, (int)std::floor((spawnPoint.second + requiredDistance) / scale));
                int startCol = std::max(0, (int)std::floor((spawnPoint.first - requiredDistance) / scale));
                int endCol = std::min(gridCols - 1, (int)std::floor((spawnPoint.first + requiredDistance) / scale));

                float minDist = std::numeric_limits<float>::max();
                std::pair<float, float> closestWallCenter;
                for (int r = startRow; r <= endRow; r++) {
                    for (int c = startCol; c <= endCol; c++) {
                        if (known_grid.occupancy[r][c] == 0) {  // wall cell
                            float cellCenterX = (c + 0.5f) * scale;
                            float cellCenterY = (r + 0.5f) * scale;
                            float dist = std::sqrt((spawnPoint.first - cellCenterX) * (spawnPoint.first - cellCenterX) +
                                (spawnPoint.second - cellCenterY) * (spawnPoint.second - cellCenterY));
                            if (dist < minDist) {
                                minDist = dist;
                                closestWallCenter = { cellCenterX, cellCenterY };
                            }
                        }
                    }
                }
                if (minDist < requiredDistance) {
                    float adjust = requiredDistance - minDist;
                    float dx = spawnPoint.first - closestWallCenter.first;
                    float dy = spawnPoint.second - closestWallCenter.second;
                    float norm = std::sqrt(dx * dx + dy * dy);
                    if (norm > 0) {
                        spawnPoint.first += (dx / norm) * adjust;
                        spawnPoint.second += (dy / norm) * adjust;
                    }
                }
                // set the spawn location and mark the robot as spawned
                rr[nextRrSpawnIndex].x = spawnPoint.first;
                rr[nextRrSpawnIndex].y = spawnPoint.second;
                rr[nextRrSpawnIndex].spawned = true;
                rr[nextRrSpawnIndex].spawnTime = t;
                nextRrSpawnIndex++;
                //std::cout << "Spawned RR #" << nextRrSpawnIndex << std::endl;
                std::cout << "Spawned RR #" << nextRrSpawnIndex << " at time " << t << std::endl;
            }
            
        }

        for (auto& robot : rr) {
            if (robot.spawned && !robot.dead) {
                robot.move(consts.dt, known_grid.occupancy, rr, known_grid.occupancy, grid.occupancy, grid.foundBy, known_grid.heat, grid.heat, known_grid.height, t);
            }
            updateCounter++;
            /*if (updateCounter % 50 == 0) {
                updateInterpolatedHeatMap(grid);
                std::cout << "updated heat map" << std::endl;
            }*/

        }

        // record % over time details for plotting
        {
            int discoveredCells = 0;
            for (const auto& row : grid.foundBy) {
                for (int cell : row) {
                    if (cell != -1) discoveredCells++;
                }
            }
            int totalCells = grid.foundBy.size() * grid.foundBy[0].size();
            float coveragePercent = (static_cast<float>(discoveredCells) / totalCells) * 100.0f;
            coverageHistory.push_back({ t, coveragePercent });
        }

        t += consts.dt;
        lastVineReversingState = vr.reversing;
        return t >= consts.maxTime;
    }


    void initializeHeatMap(float initTime, float baseTemp) {
        int rows = known_grid.heat.size();
        if (rows == 0) return;
        int cols = known_grid.heat[0].size();

        // set every cell to the ambient base temperature
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                known_grid.heat[i][j] = baseTemp;
            }
        }

        // simulate the heat diffusion for the given initialization time
        float accumulatedTime = 0.0f;
        while (accumulatedTime < initTime) {
            heatmapUpdate(known_grid.occupancy, known_grid.heat, consts.dt, consts.cellSize);
            injectHeatSources(known_grid.heat, personPositions, 37.0f, consts.cellSize);
            accumulatedTime += consts.dt;
        }
    }
};