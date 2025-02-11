#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>
#include <iostream>
#include <vector>
#include <cmath>    // For cosf() and sinf()
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <random>

// Window size
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 800;

// A constant for pi.
constexpr float PI = 3.14159265f;

// GLFW error callback
void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error (" << error << "): " << description << std::endl;
}

// Definition for RescueRobot.
struct RescueRobot {
    float x, y, theta;   // Position and orientation (in world units)
    float size;          // Robot size in meters (should be about 0.15m)
    float vx, vy;        // Velocity components
    float battery;       // Battery level
    int time_drop;       // Time when the robot dropped (if applicable)
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

// Function to set up a sample occupancy grid.
void setSampleGrid(Grid& grid) {
    int rows = grid.occupancy.size();
    if (rows == 0) return;
    int cols = grid.occupancy[0].size();

    // 1. Fill the grid with ground (value 1).
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x)
            grid.occupancy[y][x] = 1;

    // 2. Create walls (value 0) along the border.
    for (int x = 0; x < cols; ++x) {
        grid.occupancy[0][x] = 0;
        grid.occupancy[rows - 1][x] = 0;
    }
    for (int y = 0; y < rows; ++y) {
        grid.occupancy[y][0] = 0;
        grid.occupancy[y][cols - 1] = 0;
    }

    // 3. Add a couple of holes (value 2) inside.
    if (rows > 4 && cols > 4) {
        grid.occupancy[2][2] = 2;
        grid.occupancy[rows / 2][cols / 2] = 2;
    }
}

// Simulation class that contains two grids and a vector of RescueRobot objects.
class Simulation {
public:
    Grid known_grid;
    Grid grid;
    std::vector<RescueRobot> rr;

    // The constructor takes grid dimensions/scale, number of robots, and sensor count.
    Simulation(int grid_rows, int grid_cols, float scale_m, int n_robots, int robot_sensor_count)
        : known_grid(grid_rows, grid_cols, scale_m),
        grid(grid_rows, grid_cols, scale_m),
        rr(n_robots)
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
            robot.vx = 0.0f;
            robot.vy = 0.0f;
            robot.battery = 100.0f;
            robot.time_drop = 0;
            robot.sensors.resize(robot_sensor_count, 0);
            robot.size = 0.15f; // Set robot size to 0.15 meters.
        }
    }

    // Update method: advances the simulation by dt seconds.
    // Each robot will update its orientation slightly (random change) and then
    // move at a constant speed. They will bounce off the grid boundaries.
    void update(float dt) {
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
            robot.vx = speed * cosf(robot.theta);
            robot.vy = speed * sinf(robot.theta);
            // Update position.
            robot.x += robot.vx * dt;
            robot.y += robot.vy * dt;
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
    }
};

// -----------------------------------------------------------------------------
// Function: renderRobots
// Renders each robot in the simulation as a small triangle whose tip points in
// the direction of its theta. The robot positions (in world coordinates) are
// converted to screen coordinates using the provided scaleFactor.
// -----------------------------------------------------------------------------
void renderRobots(const Simulation& simulation, float scaleFactor) {
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
void renderGrid(const Simulation& simulation, float scaleFactor) {
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

int main() {
    // Initialize GLFW.
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    // Create an OpenGL 1.0 window.
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "OpenGL 1.0 + ImGui", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize ImGui.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    // Create a Simulation instance.
    // For example: a 100x100 grid with cells of 0.1 meters, 3 robots, and each robot has 5 sensors.
    Simulation simulation(100, 100, 0.1f, 300, 5);
    setSampleGrid(simulation.known_grid);

    // Compute the scale factor so that the entire grid fits the window.
    int gridCols = simulation.known_grid.occupancy[0].size();
    float scaleFactor = float(WINDOW_WIDTH) / (gridCols * simulation.known_grid.scale_m);

    // Create a mutex and an atomic flag for running the simulation update thread.
    std::mutex simMutex;
    std::atomic<bool> running(true);

    // Start the simulation update thread.
    std::thread simThread(
        [&simulation, &simMutex, &running]() {
        const float dt = 0.01f; // 10 ms time step
        while (running.load()) {
            {
                std::lock_guard<std::mutex> lock(simMutex);
                simulation.update(dt);
            }
            //std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
       }
    );

    // Main rendering loop.
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        glClear(GL_COLOR_BUFFER_BIT);

        // Set up an orthographic projection so that our world coordinates map directly to screen pixels.
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        // Lock the simulation state during rendering.
        {
            std::lock_guard<std::mutex> lock(simMutex);
            renderGrid(simulation, scaleFactor);
            renderRobots(simulation, scaleFactor);
        }

        // Restore matrices.
        glPopMatrix(); // Modelview
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);

        glfwSwapBuffers(window);
    }

    // Signal the simulation thread to stop and join.
    running.store(false);
    simThread.join();

    // Cleanup.
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
