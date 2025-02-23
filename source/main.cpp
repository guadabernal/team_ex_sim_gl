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
#include "simul.hpp"
#include "simul_render.hpp"
#include "rescue_robot.hpp"
#include <Windows.h>

// Window size
const int WINDOW_WIDTH = 1800; // increased to accommodate 3 columns
const int WINDOW_HEIGHT = 1200;
const int numCols = 3;
const int numRows = 2;
int colWidth = WINDOW_WIDTH / numCols;  // e.g., 600 pixels per column
int rowHeight = WINDOW_HEIGHT / numRows;  // e.g., 600 pixels per row

// GLFW error callback
void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error (" << error << "): " << description << std::endl;
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
    SimConsts simConsts;
    simConsts.cellSize = 0.05f;
    simConsts.totalSize = 20.0f;
    simConsts.nRobots = 10;
    simConsts.muHoleSize = 0.5f;
    simConsts.sigmaHoleSize = 0.2f;
    simConsts.nHoles = 8;
    simConsts.nPeople = 2;
    simConsts.maxTime = 30.0f;
    simConsts.dt = 0.01f;

    // --------------- Create Simulation instance --------------------
    Simulation simulation(simConsts);

    
    // --------------- Run Simulation & Plot--------------------

    // Compute the scale factor so that the 4 grids fits the window.
    float viewWidth = WINDOW_WIDTH / 3.0f;
    float renderScaleFactor = (float)colWidth / simulation.consts.totalSize;
    //float renderScaleFactor = float(WINDOW_WIDTH) / (total_size);

    std::mutex simMutex;
    std::atomic<bool> running(true);
    bool simulationEnded = false;

    std::thread simThread([&simulation, &simMutex, &running, &simulationEnded]() {
        DWORD_PTR simAffinityMask = 0x4; // CPU 1 (assuming 0-indexed cores)
        HANDLE hSimThread = GetCurrentThread(); // Get current thread handle

        DWORD_PTR previousMask = SetThreadAffinityMask(hSimThread, simAffinityMask);
        if (previousMask == 0) {
            std::cerr << "Failed to set affinity for simulation thread." << std::endl;
        }
        else {
            std::cout << "Simulation thread affinity set successfully." << std::endl;
        }

        auto lastRenderTime = std::chrono::steady_clock::now();
        bool done = false;
        while (running.load() && !done) {
            {
                std::lock_guard<std::mutex> lock(simMutex);
                done = simulation.update();
            }
            if (done) simulationEnded = true;
            auto now = std::chrono::steady_clock::now();
            auto renderElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastRenderTime);

            if (renderElapsed > std::chrono::milliseconds(50)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                //std::this_thread::sleep_for(std::chrono::nanoseconds(500));
                lastRenderTime = std::chrono::steady_clock::now();
            }
            //std::this_thread::sleep_for(std::chrono::nanoseconds(100));
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
            glViewport(0, rowHeight, colWidth, rowHeight);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            renderGrid(simulation, renderScaleFactor);
            renderRobots(simulation, renderScaleFactor);
            renderVineRobot(simulation, renderScaleFactor);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();

            // Left Bottom: True heat map.
            glViewport(0, 0, colWidth, rowHeight);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            renderHeatMap(simulation, renderScaleFactor);
            renderRobots(simulation, renderScaleFactor);
            renderVineRobot(simulation, renderScaleFactor);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();


            // Right Top: Discovered occupancy grid with robots (lidar view).
            glViewport(colWidth, rowHeight, colWidth, rowHeight);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            renderMeasurementGrid(simulation, renderScaleFactor);
            renderRobots(simulation, renderScaleFactor);
            renderVineRobot(simulation, renderScaleFactor);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();

            // Right Bottom: Discovered heat map from the robots' heat sensor.
            glViewport(colWidth, 0, colWidth, rowHeight);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, viewWidth, WINDOW_HEIGHT / 2, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            renderDiscoveredHeatMap(simulation, renderScaleFactor);
            renderRobots(simulation, renderScaleFactor);
            renderVineRobot(simulation, renderScaleFactor);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();

            //heat map
            glViewport(2 * colWidth, 0, colWidth, rowHeight);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            renderInterpolatedHeatMap(simulation, renderScaleFactor); // New function (see below)
            renderRobots(simulation, renderScaleFactor);
            renderVineRobot(simulation, renderScaleFactor);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();

            //Incline map
            glViewport(2 * colWidth, rowHeight, colWidth, rowHeight);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            renderRobots(simulation, renderScaleFactor);
            renderVineRobot(simulation, renderScaleFactor);
            renderInclineMap(simulation, renderScaleFactor);
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
        float runtime = robot.dead ? (robot.timeDeath - robot.spawnTime) : (simulation.t - robot.spawnTime);
        
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

    int vineCount = 0;
    for (const auto& row : simulation.grid.foundBy) {
        for (int cell : row) {
            if (cell == -2)
                vineCount++;
        }
    }
    float vinePercent = (static_cast<float>(vineCount) / totalCells) * 100.0f;
    std::cout << "Vine Robot discovered " << vinePercent << "% of the area." << std::endl;


    float totalCoveragePercent = (static_cast<float>(coveredCells) / totalCells) * 100.0f;
    std::cout << "Total area coverage by rescue robots: " << totalCoveragePercent << "%" << std::endl;


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
        glViewport(0, rowHeight, colWidth, rowHeight);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        renderGrid(simulation, renderScaleFactor);
        renderRobots(simulation, renderScaleFactor);
        renderVineRobot(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        // Left Bottom: True heat map.
        glViewport(0, 0, colWidth, rowHeight);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        renderHeatMap(simulation, renderScaleFactor);
        renderRobots(simulation, renderScaleFactor);
        renderVineRobot(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        // --------------------
        // Right side viewports (split into two vertical sections)
        // --------------------

        // Right Top: Discovered occupancy grid with robots (lidar view).
        glViewport(colWidth, rowHeight, colWidth, rowHeight);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        renderMeasurementGrid(simulation, renderScaleFactor);
        renderRobots(simulation, renderScaleFactor);
        renderVineRobot(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        // Right Bottom: Discovered heat map from the robots' heat sensor.
        glViewport(colWidth, 0, colWidth, rowHeight);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        renderDiscoveredHeatMap(simulation, renderScaleFactor);
        renderRobots(simulation, renderScaleFactor);
        renderVineRobot(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        // heat map
        glViewport(2 * colWidth, 0, colWidth, rowHeight);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        renderInterpolatedHeatMap(simulation, renderScaleFactor); // New function (see below)
        renderRobots(simulation, renderScaleFactor);
        renderVineRobot(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        glViewport(2 * colWidth, rowHeight, colWidth, rowHeight);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        renderRobots(simulation, renderScaleFactor);
        renderVineRobot(simulation, renderScaleFactor);
        renderInclineMap(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        // Optionally, use ImGui to display sensor data or additional info.
        //ImGui_ImplOpenGL2_NewFrame();
        //ImGui_ImplGlfw_NewFrame();
        //ImGui::NewFrame();

        //ImGui::Begin("Final Sensor Data");
        //for (const auto& robot : simulation.rr) {
        //    ImGui::Text("Robot %d:", robot.id);
        //    for (size_t i = 0; i < robot.sensors.size(); ++i) {
        //        ImGui::Text("Sensor %zu: %f", i, robot.sensors[i]);
        //        ImGui::SameLine();
        //    }
        //    ImGui::Separator();
        //}

        //ImGui::End();
        //ImGui::Render();

        glfwSwapBuffers(finalWindow);
    }

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(finalWindow);
    glfwTerminate();

    return 0;
}
