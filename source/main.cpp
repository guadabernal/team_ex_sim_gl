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
#include "gen_occupancy.hpp"
#include "simul.hpp"
#include "simul_render.hpp"

// Window size
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 800;

// GLFW error callback
void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error (" << error << "): " << description << std::endl;
}

std::vector<RescueRobot> spawnRobots(const Simulation& simulation, int nRobots, int sensorCount) {
    std::vector<RescueRobot> robots;
    robots.resize(nRobots);

    // Determine grid dimensions and cell size.
    int gridRows = simulation.known_grid.occupancy.size();
    int gridCols = simulation.known_grid.occupancy[0].size();
    float cellSize = simulation.known_grid.scale_m;

    std::default_random_engine rng(std::random_device{}());
    std::uniform_real_distribution<float> posXDist(0.0f, gridCols * cellSize);
    std::uniform_real_distribution<float> posYDist(0.0f, gridRows * cellSize);
    std::uniform_real_distribution<float> thetaDist(0.0f, 2 * PI);

    // Use the robot size as the padding distance.
    float spawnPadding = 0.12f; // Adjust if robot size differs.

    // For each robot, repeatedly sample until a valid (padded) position is found.
    for (auto& robot : robots) {
        bool valid = false;
        while (!valid) {
            float candidateX = posXDist(rng);
            float candidateY = posYDist(rng);

            // Compute the bounding box for the candidate spawn point with padding.
            float left = candidateX - spawnPadding;
            float right = candidateX + spawnPadding;
            float top = candidateY - spawnPadding;
            float bottom = candidateY + spawnPadding;

            int col_min = static_cast<int>(std::floor(left / cellSize));
            int col_max = static_cast<int>(std::floor(right / cellSize));
            int row_min = static_cast<int>(std::floor(top / cellSize));
            int row_max = static_cast<int>(std::floor(bottom / cellSize));

            // Clamp indices to grid bounds.
            if (col_min < 0) col_min = 0;
            if (row_min < 0) row_min = 0;
            if (col_max >= gridCols) col_max = gridCols - 1;
            if (row_max >= gridRows) row_max = gridRows - 1;

            // Check that every cell in the padded bounding box is valid ground.
            valid = true;
            for (int row = row_min; row <= row_max && valid; ++row) {
                for (int col = col_min; col <= col_max && valid; ++col) {
                    if (simulation.known_grid.occupancy[row][col] != 1) {
                        valid = false;
                    }
                }
            }

            if (valid) {
                robot.x = candidateX;
                robot.y = candidateY;
            }
        }
        robot.theta = thetaDist(rng);
        // Note: v should be set during spawning and not in move().
        // Here, we set it once (adjust as needed).
        robot.v = 0.2f;
        robot.battery = 100.0f;
        robot.time_drop = 0;
        robot.sensors.resize(sensorCount, 0);
        robot.size = 0.12f;
    }
    return robots;
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
    float cell_size = 0.05f; // meters
    float total_size = 10; // meters
    int grid_size = total_size / cell_size;
    
    int nRobots = 50;      // desired number of robots
    int sensorCount = 5;   // sensor count per robot
    
    Simulation simulation(grid_size, grid_size, cell_size, nRobots, sensorCount, 100, 0.01);
    
    float muHoleSize = 0.15;
    float sigmaHoleSize = 0.02;

    
    generateOfficeMap(simulation.known_grid.occupancy, cell_size, 0.15f, 0.9f); // first place walls
    addHoles(simulation.known_grid.occupancy, cell_size, muHoleSize, sigmaHoleSize); // second add holes
    // generateSensorX(...)

    simulation.rr = spawnRobots(simulation, nRobots, sensorCount);

    


    // Compute the scale factor so that the entire grid fits the window.
    float renderScaleFactor = float(WINDOW_WIDTH) / (total_size);

    // Create a mutex and an atomic flag for running the simulation update thread.
    std::mutex simMutex;
    std::atomic<bool> running(true);

    // Start the simulation update thread.
    std::thread simThread(
        [&simulation, &simMutex, &running]() {        
        bool done = false;
        while (running.load() && !done) {
            {
                std::lock_guard<std::mutex> lock(simMutex);
                done = simulation.update();
            }
             std::this_thread::sleep_for(std::chrono::microseconds(1)); // comment for visualization 
        }
        running.store(false);
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
            renderGrid(simulation, renderScaleFactor);
            renderRobots(simulation, renderScaleFactor);
        }

        // Restore matrices.
        glPopMatrix(); // Modelview
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);

        glfwSwapBuffers(window);
        if (!running.load()) {
            std::cout << "Finishing everything" << std::endl;
            running.store(true);
            // break; // break if wanted to stop the visualization
        }
    }

    // Signal the simulation thread to stop and join.
    running.store(false);
    simThread.join();
    std::cout << "Finishing everything" << std::endl;
    // Cleanup.
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
