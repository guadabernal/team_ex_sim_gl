#pragma once

constexpr float PI = 3.14159265f;

struct RescueRobot {
    float x, y, theta;
    float size;
    float v;
    float battery;
    int time_drop;
    float time_death = 0.0f;

    std::vector<int> sensors;  // Sensor readings
    std::vector<float> sensorLastUpdateTimes;

    bool dead = false;         // Indicates if the robot fell in a hole
    bool rotating = false;     // Indicates if the robot is trying to rotate in place

    int id;

    void measure(const std::vector<std::vector<int>>& trueOccupancy, std::vector<std::vector<int>>& unknownOccupancy, std::vector<std::vector<int>>& foundBy, 
        const std::vector<std::vector<float>>& trueHeat, std::vector<std::vector<float>>& knownHeat, float cellSize, float currentTime)
    {
        // ----- Distance Sensor Measurement (unchanged) -----
        const float defaultUpdateInterval = 0.1f;
        for (size_t i = 0; i < sensors.size(); ++i) {
            if (currentTime - sensorLastUpdateTimes[i] < defaultUpdateInterval)
                continue; // Not time yet for this sensor.
            std::vector<float> measurements = sensor_lidar(trueOccupancy, cellSize);
            if (measurements.empty())
                continue;
            float distance = measurements[0];
            const float maxRange = 4.0f;
            int steps = static_cast<int>(distance / cellSize);
            for (int j = 0; j < steps; j++) {
                float rayX = x + (j * cellSize) * std::cos(theta);
                float rayY = y + (j * cellSize) * std::sin(theta);
                int col = static_cast<int>(rayX / cellSize);
                int row = static_cast<int>(rayY / cellSize);
                if (row >= 0 && row < unknownOccupancy.size() &&
                    col >= 0 && col < unknownOccupancy[0].size()) {
                    if (unknownOccupancy[row][col] == -1) { // Only update if not discovered yet.
                        unknownOccupancy[row][col] = 1; // ground
                        foundBy[row][col] = id;
                    }
                }
            }
            if (distance < maxRange) {
                float sensorX = x + distance * std::cos(theta);
                float sensorY = y + distance * std::sin(theta);
                int col = static_cast<int>(sensorX / cellSize);
                int row = static_cast<int>(sensorY / cellSize);
                if (row >= 0 && row < unknownOccupancy.size() &&
                    col >= 0 && col < unknownOccupancy[0].size()) {
                    if (unknownOccupancy[row][col] == -1) {
                        unknownOccupancy[row][col] = 0; // wall
                        foundBy[row][col] = id;
                    }
                }
            }
            sensorLastUpdateTimes[i] = currentTime;
        }

        // ----- Heat Sensor Measurement -----
        // Update only the cell where the robot is located with the direct temperature reading.
        int col = static_cast<int>(x / cellSize);
        int row = static_cast<int>(y / cellSize);
        if (row >= 0 && row < knownHeat.size() &&
            col >= 0 && col < knownHeat[0].size()) {
            knownHeat[row][col] = sensor_heat(trueHeat, cellSize);
        }
    }


    int checkCollision(const std::vector<std::vector<int>>& occupancy, float cellSize, const std::vector<RescueRobot>& robots) {
        float halfSize = size / 2.0f;
        float left = x - halfSize;
        float right = x + halfSize;
        float top = y - halfSize;
        float bottom = y + halfSize;

        // Convert world coordinates to grid indices.
        int col_min = static_cast<int>(std::floor(left / cellSize));
        int col_max = static_cast<int>(std::floor(right / cellSize));
        int row_min = static_cast<int>(std::floor(top / cellSize));
        int row_max = static_cast<int>(std::floor(bottom / cellSize));

        int rows = occupancy.size();
        int cols = occupancy[0].size();
        if (col_min < 0) col_min = 0;
        if (row_min < 0) row_min = 0;
        if (col_max >= cols) col_max = cols - 1;
        if (row_max >= rows) row_max = rows - 1;

        bool wallFound = false;
        bool holeFound = false;
        // Check occupancy grid.
        for (int row = row_min; row <= row_max; ++row) {
            for (int col = col_min; col <= col_max; ++col) {
                int cell = occupancy[row][col];
                if (cell != 1) { // Not ground.
                    if (cell == 0)
                        wallFound = true;
                    else if (cell == 2)
                        holeFound = true;
                }
            }
        }
        if (wallFound) return 1;
        if (holeFound) return 2;

        // Check collision with other robots.
        for (const auto& other : robots) {
            // Skip self.
            if (&other == this) continue;
            float otherHalf = other.size / 2.0f;
            float otherLeft = other.x - otherHalf;
            float otherRight = other.x + otherHalf;
            float otherTop = other.y - otherHalf;
            float otherBottom = other.y + otherHalf;
            // Check for overlap using axis-aligned bounding box (AABB) collision.
            bool overlapX = (x - halfSize < otherRight) && (x + halfSize > otherLeft);
            bool overlapY = (y - halfSize < otherBottom) && (y + halfSize > otherTop);
            if (overlapX && overlapY) return 1;
        }
        return 0;
    }

    void move(float dt, const std::vector<std::vector<int>>& occupancy, float cellSize, const std::vector<RescueRobot>& robots,
        const std::vector<std::vector<int>>& trueOccupancy, std::vector<std::vector<int>>& unknownOccupancy, std::vector<std::vector<int>>& foundBy,
        const std::vector<std::vector<float>>& trueHeat, std::vector<std::vector<float>>& knownHeat, float currentTime)
    {
        if (currentTime < time_drop || dead) return;
        
        float prevX = x, prevY = y;

        if (rotating) {
            const float ROTATE_DELTA = 0.1f;
            theta += ROTATE_DELTA;
        }
        else {
            static std::default_random_engine rng(std::random_device{}());
            static std::uniform_real_distribution<float> angleDist(-0.1f, 0.1f);
            theta += angleDist(rng);
        }

        float candidateX = x + v * std::cos(theta) * dt;
        float candidateY = y + v * std::sin(theta) * dt;
        x = candidateX;
        y = candidateY;

        int collision = checkCollision(occupancy, cellSize, robots);

        if (collision == 0) {
            rotating = false;
        }
        else if (collision == 1) {
            x = prevX;
            y = prevY;
            rotating = true;
            return;
        }
        else if (collision == 2) {
            v = 0;
            dead = true;
            time_death = currentTime;  // Record the actual death time.
            return;
        }

        // Update the discovered occupancy grid.
        measure(occupancy, unknownOccupancy, foundBy, trueHeat, knownHeat, cellSize, currentTime);

    }

    
    // --- Sensor Reading Functions Below -----
    float sensor_heat(const std::vector<std::vector<float>>& trueHeat, float cellSize) {
        int col = static_cast<int>(x / cellSize);
        int row = static_cast<int>(y / cellSize);
        // If the robot is out of bounds, return the base temperature (e.g., 20�C).
        if (row < 0 || row >= trueHeat.size() || col < 0 || col >= trueHeat[0].size())
            return 20.0f;
        float temp = trueHeat[row][col];
        // Clamp the temperature to a realistic range.
        if (temp < -40.0f)
            temp = -40.0f;
        if (temp > 125.0f)
            temp = 125.0f;
        return temp;
    }

    std::vector<float> sensor_lidar(const std::vector<std::vector<int>>& occupancy, float cellSize) {
        const float max_range = 2.0f;
        float step = cellSize * 0.5f;
        float distance = 0.0f;
        while (distance < max_range) {
            float rayX = x + distance * std::cos(theta);
            float rayY = y + distance * std::sin(theta);
            int col = static_cast<int>(rayX / cellSize);
            int row = static_cast<int>(rayY / cellSize);
            if (row < 0 || row >= occupancy.size() ||
                col < 0 || col >= occupancy[0].size()) {
                break; // outside grid bounds
            }
            // Only treat walls (occupancy == 0) as obstacles.
            if (occupancy[row][col] == 0) {
                return { distance };
            }
            // Ground (1) and holes (2) are ignored by the sensor.
            distance += step;
        }
        return { max_range };
    }

    float sensor_co2() { return 0; }

    int sensor_aqi() { return 0; }
};

struct VineRobot {
    float x, y, theta;         // initial position and orientation (in world units)
    void move() {}
};

struct Grid {
    float scale_m;  // Size of each cell in meters
    std::vector<std::vector<float>> co2;
    std::vector<std::vector<float>> heat;
    std::vector<std::vector<int>> ground_type;
    std::vector<std::vector<float>> incline;
    std::vector<std::vector<int>> occupancy;
    std::vector<std::vector<int>> foundBy;

    // Constructor: initializes all maps with 'rows' x 'cols' elements.
    Grid(int rows, int cols, float scale_m)
        : co2(rows, std::vector<float>(cols, 0.0f)),
        heat(rows, std::vector<float>(cols, 0.0f)),
        ground_type(rows, std::vector<int>(cols, 0)),
        incline(rows, std::vector<float>(cols, 0.0f)),
        occupancy(rows, std::vector<int>(cols, 0)),
        foundBy(rows, std::vector<int>(cols, -1)),
        scale_m(scale_m)
    {}
};


inline void heatmapUpdate(const std::vector<std::vector<int>>& occupancy,
    std::vector<std::vector<float>>& heat,
    float dt,
    float dx)
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

inline void injectHeatSources(std::vector<std::vector<float>>& heat,
    int numOfPpl,
    const std::vector<std::pair<float, float>>& sourcePositions,
    float personTemp,
    float scale)
{
    int rows = heat.size();
    if (rows == 0) return;
    int cols = heat[0].size();

    // Person diameter is 30cm; therefore the radius is 15cm (0.15m).
    float personRadius = 0.15f; // in meters
    // Compute how many cells correspond to the person radius.
    int radiusCells = static_cast<int>(std::ceil(personRadius / scale));

    // For each heat source...
    for (int i = 0; i < numOfPpl && i < sourcePositions.size(); i++) {
        // The center of the person is given in grid coordinates.
        int centerX = static_cast<int>(sourcePositions[i].first);
        int centerY = static_cast<int>(sourcePositions[i].second);

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

// Simulation class that contains two grids and a vector of RescueRobot objects.
class Simulation {
public:
    Grid known_grid;
    Grid grid;
    
    std::vector<RescueRobot> rr;
    //VineRobot vr;

    float t = 0;
    float dt;
    float max_time;
    int numOfPpl;
    std::vector<std::pair<float, float>> sourcePositions;

    Simulation(int grid_rows, int grid_cols, float scale_m, int n_robots, int robot_sensor_count, float max_time, float dt = 0.01f, int numOfPpl = 0,
        const std::vector<std::pair<float, float>>& sourcePositions = std::vector<std::pair<float, float>>())
        : known_grid(grid_rows, grid_cols, scale_m), grid(grid_rows, grid_cols, scale_m), rr(n_robots), max_time(max_time),
        dt(dt), numOfPpl(numOfPpl), sourcePositions(sourcePositions)
    {}

    // Update method: advances the simulation by dt seconds.
    // Each robot will update its orientation slightly (random change) and then
    // move at a constant speed. They will bounce off the grid boundaries.
    bool update() {
        heatmapUpdate(known_grid.occupancy, known_grid.heat, dt, known_grid.scale_m);
        injectHeatSources(known_grid.heat, numOfPpl, sourcePositions, 37.0f, known_grid.scale_m);



        for (auto& robot : rr) {
            robot.move(dt,
                known_grid.occupancy,       // occupancy grid
                known_grid.scale_m,         // cell size
                rr,                         // list of robots
                known_grid.occupancy,       // trueOccupancy
                grid.occupancy,             // unknownOccupancy (discovered occupancy)
                grid.foundBy,               // foundBy grid
                known_grid.heat,            // trueHeat map
                grid.heat,                  // knownHeat map (discovered heat)
                t);                         // currentTime
        }
        t += dt;
        return t >= max_time;
    }

    void initializeHeatMap(float initTime, float baseTemp) {
        int rows = known_grid.heat.size();
        if (rows == 0) return;
        int cols = known_grid.heat[0].size();

        // Set every cell to the ambient base temperature.
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                known_grid.heat[i][j] = baseTemp;
            }
        }

        // Simulate the heat diffusion for the given initialization time.
        float accumulatedTime = 0.0f;
        while (accumulatedTime < initTime) {
            // Diffuse the heat map using the current occupancy information.
            heatmapUpdate(known_grid.occupancy, known_grid.heat, dt, known_grid.scale_m);
            // Re-inject heat sources so they remain at personTemp.
            injectHeatSources(known_grid.heat, numOfPpl, sourcePositions, 37.0f, known_grid.scale_m);
            accumulatedTime += dt;
        }
    }
};