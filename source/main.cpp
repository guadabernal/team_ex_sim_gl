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

std::vector<RescueRobot> spawnRobots(Simulation& simulation, int nRobots, int sensorCount) {
    std::vector<RescueRobot> robots;
    robots.resize(nRobots);

    int gridRows = simulation.known_grid.occupancy.size();
    int gridCols = simulation.known_grid.occupancy[0].size();
    float cellSize = simulation.known_grid.scale_m;

    std::default_random_engine rng(std::random_device{}());
    std::uniform_real_distribution<float> posXDist(0.0f, gridCols * cellSize);
    std::uniform_real_distribution<float> posYDist(0.0f, gridRows * cellSize);
    std::uniform_real_distribution<float> thetaDist(0.0f, 2 * PI);

    float spawnPadding = 0.12f; // robot size as padding

    for (int i = 0; i < robots.size(); i++) {
        RescueRobot& robot = robots[i];
        bool valid = false;
        while (!valid) {
            float candidateX = posXDist(rng);
            float candidateY = posYDist(rng);

            float left = candidateX - spawnPadding;
            float right = candidateX + spawnPadding;
            float top = candidateY - spawnPadding;
            float bottom = candidateY + spawnPadding;

            int col_min = static_cast<int>(std::floor(left / cellSize));
            int col_max = static_cast<int>(std::floor(right / cellSize));
            int row_min = static_cast<int>(std::floor(top / cellSize));
            int row_max = static_cast<int>(std::floor(bottom / cellSize));

            if (col_min < 0) col_min = 0;
            if (row_min < 0) row_min = 0;
            if (col_max >= gridCols) col_max = gridCols - 1;
            if (row_max >= gridRows) row_max = gridRows - 1;

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
        robot.v = 0.2f;
        robot.battery = 100.0f;
        robot.time_drop = 0;
        robot.sensors.resize(sensorCount, 0);
        robot.size = 0.12f;
        robot.id = i;
        robot.sensorLastUpdateTimes.resize(sensorCount, 0.0f);
    }
    return robots;
}



int main() {
    // Initialize GLFW
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    // Create an OpenGL 1.0 window
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "OpenGL 1.0 + ImGui", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);


    // --------------- Create Simulation consts --------------------

    float cell_size = 0.05f; // meters
    float total_size = 10;    // meters
    int grid_size = total_size / cell_size;
    int nRobots = 10;         // number of robots
    int sensorCount = 5;      // sensor count per robot
    
    
    float muHoleSize = 0.5;
    float sigmaHoleSize = 0.2;
    int numHoles = 8;

    int numOfPpl = 1;         // number of heat sources


    std::vector<std::pair<float, float>> personPositions;
    int gridCols = grid_size;
    int gridRows = grid_size;
    personPositions.push_back({ gridCols * 0.25f, gridRows * 0.1f });/*
    personPositions.push_back({ gridCols * 0.8f, gridRows * 0.2f });
    personPositions.push_back({ gridCols * 0.2f, gridRows * 0.8f });
    personPositions.push_back({ gridCols * 0.8f, gridRows * 0.8f });
    personPositions.push_back({ gridCols * 0.5f, gridRows * 0.2f });
    personPositions.push_back({ gridCols * 0.5f, gridRows * 0.8f });
    personPositions.push_back({ gridCols * 0.2f, gridRows * 0.5f });
    personPositions.push_back({ gridCols * 0.8f, gridRows * 0.5f });
    personPositions.push_back({ gridCols * 0.35f, gridRows * 0.35f });
    personPositions.push_back({ gridCols * 0.65f, gridRows * 0.65f });*/



    // --------------- Create Simulation instance --------------------
    Simulation simulation(grid_size, grid_size, cell_size, nRobots, sensorCount, 30, 0.01f, numOfPpl, personPositions);

    // --------------- Fill Simulation Maps --------------------

    generateOfficeMap(simulation.known_grid.occupancy, cell_size, 0.15f, 0.9f); // first place walls
    addHoles(simulation.known_grid.occupancy, cell_size, muHoleSize, sigmaHoleSize, numHoles); // second add holes
    simulation.initializeHeatMap(10.0f, 20.0f);

    for (auto& row : simulation.grid.occupancy) {
        std::fill(row.begin(), row.end(), -1);
    }

    simulation.rr = spawnRobots(simulation, nRobots, sensorCount);

    
    // --------------- Run Simulation & Plot--------------------

    // Compute the scale factor so that the 4 grids fits the window.
    float viewWidth = WINDOW_WIDTH / 2.0f;
    float renderScaleFactor = viewWidth / total_size;
    //float renderScaleFactor = float(WINDOW_WIDTH) / (total_size);

    std::mutex simMutex;
    std::atomic<bool> running(true);
    bool simulationEnded = false;

    std::thread simThread([&simulation, &simMutex, &running, &simulationEnded]() {
        bool done = false;
        while (running.load() && !done) {
            {
                std::lock_guard<std::mutex> lock(simMutex);
                done = simulation.update();
            }
            if (done) simulationEnded = true;

            /*std::this_thread::sleep_for(std::chrono::microseconds(1));*/
            //std::this_thread::sleep_for(std::chrono::nanoseconds(500));
        }
        running.store(false);
        });


    // Main rendering loop.
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT);

        // Render viewports
        {
            std::lock_guard<std::mutex> lock(simMutex);

            
            // Left Top: True occupancy grid with robots.
            glViewport(0, WINDOW_HEIGHT / 2, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, viewWidth, WINDOW_HEIGHT / 2, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            renderGrid(simulation, renderScaleFactor);
            renderRobots(simulation, renderScaleFactor);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();

            // Left Bottom: True heat map.
            glViewport(0, 0, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, viewWidth, WINDOW_HEIGHT / 2, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            renderHeatMap(simulation, renderScaleFactor);
            renderRobots(simulation, renderScaleFactor);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();


            // Right Top: Discovered occupancy grid with robots (lidar view).
            glViewport(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, viewWidth, WINDOW_HEIGHT / 2, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            renderMeasurementGrid(simulation, renderScaleFactor);
            renderRobots(simulation, renderScaleFactor);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();

            // Right Bottom: Discovered heat map from the robots' heat sensor.
            glViewport(WINDOW_WIDTH / 2, 0, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, viewWidth, WINDOW_HEIGHT / 2, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            renderDiscoveredHeatMap(simulation, renderScaleFactor);
            renderRobots(simulation, renderScaleFactor);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();
        }

        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::Render();
        glfwSwapBuffers(window);
        if (simulationEnded) {
            std::cout << "Closing Simulation" << std::endl;
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
    }


    // Signal the simulation thread to stop (if not already done) and join it.
    running.store(false);
    simThread.join();

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);



    // --------------- Output Results -----------------------------

    int totalCells = simulation.grid.foundBy.size() * simulation.grid.foundBy[0].size();
    int coveredCells = 0;
    
    for (auto& robot : simulation.rr) {
        
        // Determine run time: if the robot died, use its time_drop; otherwise, the current simulation time.
        float runtime = robot.dead ? (robot.time_death - robot.time_drop) : (simulation.t - robot.time_drop);
        
        int discoveredCount = 0;
        for (const auto& row : simulation.grid.foundBy) {
            for (int cell : row) {
                if (cell == robot.id) discoveredCount++;
            }
        }
        coveredCells += discoveredCount;
        float percentExplored = (static_cast<float>(discoveredCount) / totalCells) * 100.0f;

        std::cout << "Robot " << robot.id << ": Run time = " << runtime << " sec, " << (robot.dead ? "Fell " : "Alive") << ", % area exp = " << percentExplored << "%" << std::endl;
    }

    float totalCoveragePercent = (static_cast<float>(coveredCells) / totalCells) * 100.0f;
    std::cout << "Total area coverage by all robots: " << totalCoveragePercent << "%" << std::endl;


    // ----- Create a new ImGui context for the final window -----

    GLFWwindow* finalWindow = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Final Maps", nullptr, nullptr);
    if (!finalWindow) {
        std::cerr << "Failed to create final window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(finalWindow);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(finalWindow, true);
    ImGui_ImplOpenGL2_Init();

    while (!glfwWindowShouldClose(finalWindow)) {
        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT);

        // You can reuse your rendering functions to show the final, frozen state.
        glViewport(0, WINDOW_HEIGHT / 2, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, viewWidth, WINDOW_HEIGHT / 2, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        renderGrid(simulation, renderScaleFactor);
        renderRobots(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        // Left Bottom: True heat map.
        glViewport(0, 0, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, viewWidth, WINDOW_HEIGHT / 2, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        renderHeatMap(simulation, renderScaleFactor);
        renderRobots(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        // --------------------
        // Right side viewports (split into two vertical sections)
        // --------------------

        // Right Top: Discovered occupancy grid with robots (lidar view).
        glViewport(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, viewWidth, WINDOW_HEIGHT / 2, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        renderMeasurementGrid(simulation, renderScaleFactor);
        renderRobots(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        // Right Bottom: Discovered heat map from the robots' heat sensor.
        glViewport(WINDOW_WIDTH / 2, 0, WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, viewWidth, WINDOW_HEIGHT / 2, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        renderDiscoveredHeatMap(simulation, renderScaleFactor);
        renderRobots(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        // Optionally, use ImGui to display sensor data or additional info.
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Final Sensor Data");
        for (const auto& robot : simulation.rr) {
            ImGui::Text("Robot %d:", robot.id);
            for (size_t i = 0; i < robot.sensors.size(); ++i) {
                ImGui::Text("Sensor %zu: %f", i, robot.sensors[i]);
                ImGui::SameLine();
            }
            ImGui::Separator();
        }

        ImGui::End();
        ImGui::Render();

        glfwSwapBuffers(finalWindow);
    }

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(finalWindow);
    glfwTerminate();

    return 0;
}
