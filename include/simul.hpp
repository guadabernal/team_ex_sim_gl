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

    void measure() { }
    void move() { }

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
        // Random engine setup
        std::default_random_engine rng(std::random_device{}());
        std::uniform_real_distribution<float> posXDist(0.0f, grid_cols * scale_m);
        std::uniform_real_distribution<float> posYDist(0.0f, grid_rows * scale_m);
        std::uniform_real_distribution<float> thetaDist(0.0f, 2 * PI);

        // Initialize each robot with a random position and orientation.
        for (auto& robot : rr) {
            robot.x = posXDist(rng);
            robot.y = posYDist(rng);
            robot.theta = thetaDist(rng);
            robot.v = 0.0f;
            robot.battery = 100.0f;
            robot.time_drop = 0;
            robot.sensors.resize(robot_sensor_count, 0);
            robot.size = 0.12f; 
        }
    }

    // Update method: advances the simulation by dt seconds.
    // Each robot will update its orientation slightly (random change) and then
    // move at a constant speed. They will bounce off the grid boundaries.
    bool update() {
        // Set up a random engine and a uniform distribution for a small angle change.
        static std::default_random_engine rng(std::random_device{}());
        static std::uniform_real_distribution<float> angleDist(-0.1f, 0.1f); // radians
        // Compute grid boundaries (in meters)
        float gridWidth = known_grid.occupancy[0].size() * known_grid.scale_m;
        float gridHeight = known_grid.occupancy.size() * known_grid.scale_m;
        const float speed = 0.2f; // Constant speed (m/s)

        for (auto& robot : rr) {
            // Randomly change the robot's heading.
            float dtheta = angleDist(rng);
            robot.theta += dtheta;
            // Update velocity based on the new heading.
            robot.v = speed;
            // Update position.
            robot.x += robot.v * cos(robot.theta) * dt;
            robot.y += robot.v * sin(robot.theta) * dt;
            // Bounce off the grid boundaries.
            if (robot.x < 0) {
                robot.x = 0;
                robot.theta = PI - robot.theta;
            }
            else if (robot.x > gridWidth) {
                robot.x = gridWidth;
                robot.theta = PI - robot.theta;
            }
            if (robot.y < 0) {
                robot.y = 0;
                robot.theta = -robot.theta;
            }
            else if (robot.y > gridHeight) {
                robot.y = gridHeight;
                robot.theta = -robot.theta;
            }
        }
        t += dt;
        return t >= max_time;
    }
};