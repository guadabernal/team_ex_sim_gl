#pragma once

constexpr float PI = 3.14159265f;

// Definition for RescueRobot.
struct RescueRobot {
    float x, y, theta;         // Position and orientation (in world units)
    float size;                // Robot size in meters (should be about 0.15m)
    float v;                   // Velocity components
    float battery;             // Battery level
    int time_drop;             // Time when the robot dropped (if applicable)

    std::vector<int> sensors;  // Sensor readings

    bool dead = false;         // Indicates if the robot fell in a hole
    bool rotating = false;     // Indicates if the robot is trying to rotate in place


    void measure() { }

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

    // Modified move() function that uses collision checking against the occupancy grid.
    // Parameters:
    //   dt        : time step.
    //   occupancy : occupancy grid where 0 = wall, 1 = ground, 2 = hole.
    //   cellSize  : physical size of each grid cell.
    void move(float dt, const std::vector<std::vector<int>>& occupancy, float cellSize, const std::vector<RescueRobot>& robots) {
        if (dead) return;  // Do nothing if the robot is dead.

        // Save the current valid position.
        float prevX = x, prevY = y;

        // If in rotating mode, use a constant rotation amount to turn in the same direction.
        // Otherwise, apply a small random rotation.
        if (rotating) {
            const float ROTATE_DELTA = 0.1f; // constant angle (radians)
            theta += ROTATE_DELTA;
        }
        else {
            static std::default_random_engine rng(std::random_device{}());
            static std::uniform_real_distribution<float> angleDist(-0.1f, 0.1f); // radians
            theta += angleDist(rng);
        }

        // Use the robot's current speed (v) which should have been set during spawning.
        float candidateX = x + v * std::cos(theta) * dt;
        float candidateY = y + v * std::sin(theta) * dt;

        // Temporarily update position for collision checking.
        x = candidateX;
        y = candidateY;
        int collision = checkCollision(occupancy, cellSize, robots);

        if (collision == 0) {
            // Valid move: commit candidate position and exit rotating mode.
            rotating = false;
        }
        else if (collision == 1) {
            // Collision with a wall or another robot: revert to previous position and enable rotating mode.
            x = prevX;
            y = prevY;
            rotating = true;
            return;
        }
        else if (collision == 2) {
            // Fell in a hole: stop movement and mark as dead.
            v = 0;
            dead = true;
            return;
        }
    }

    float sensor_co2() { return 0; }
    int sensor_aqi() { return 0; }
    float sensor_heat() { return 0; }
    std::vector<float> sensor_lidar() { return std::vector<float>(); }
};

// Definition for Grid.
struct Grid {
    float scale_m;  // Size of each cell in meters
    std::vector<std::vector<float>> co2;
    std::vector<std::vector<float>> heat;
    std::vector<std::vector<int>> ground_type;
    std::vector<std::vector<float>> incline;
    std::vector<std::vector<int>> occupancy;

    // Constructor: initializes all maps with 'rows' x 'cols' elements.
    Grid(int rows, int cols, float scale_m)
        : co2(rows, std::vector<float>(cols, 0.0f)),
        heat(rows, std::vector<float>(cols, 0.0f)),
        ground_type(rows, std::vector<int>(cols, 0)),
        incline(rows, std::vector<float>(cols, 0.0f)),
        occupancy(rows, std::vector<int>(cols, 0)),
        scale_m(scale_m)
    {}
};



// Simulation class that contains two grids and a vector of RescueRobot objects.
class Simulation {
public:
    Grid known_grid;
    Grid grid;
    std::vector<RescueRobot> rr;
    float t = 0;
    float dt;
    float max_time;

    // The constructor takes grid dimensions/scale, number of robots, and sensor count.
    Simulation(int grid_rows, int grid_cols, float scale_m, int n_robots, int robot_sensor_count, float max_time, float dt = 0.01f)
        : known_grid(grid_rows, grid_cols, scale_m),
        grid(grid_rows, grid_cols, scale_m),
        rr(n_robots),
        max_time(max_time),
        dt(dt)
    {
        //// Random engine setup
        //std::default_random_engine rng(std::random_device{}());
        //std::uniform_real_distribution<float> posXDist(0.0f, grid_cols * scale_m);
        //std::uniform_real_distribution<float> posYDist(0.0f, grid_rows * scale_m);
        //std::uniform_real_distribution<float> thetaDist(0.0f, 2 * PI);

        //// Initialize each robot with a random position and orientation.
        //for (auto& robot : rr) {
        //    robot.x = posXDist(rng);
        //    robot.y = posYDist(rng);
        //    robot.theta = thetaDist(rng);
        //    robot.v = 0.0f;
        //    robot.battery = 100.0f;
        //    robot.time_drop = 0;
        //    robot.sensors.resize(robot_sensor_count, 0);
        //    robot.size = 0.12f; 
        //}
    }

    // Update method: advances the simulation by dt seconds.
    // Each robot will update its orientation slightly (random change) and then
    // move at a constant speed. They will bounce off the grid boundaries.
    bool update() {
        //float gridWidth = known_grid.occupancy[0].size() * known_grid.scale_m;
        //float gridHeight = known_grid.occupancy.size() * known_grid.scale_m;

        // Update each robot.
        for (auto& robot : rr) {
            robot.move(dt, known_grid.occupancy, known_grid.scale_m, rr);
        }
        t += dt;
        return t >= max_time;
    }
};