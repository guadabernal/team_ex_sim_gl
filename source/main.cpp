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
    float total_size = 15; // meters
    int grid_size = total_size / cell_size;
    Simulation simulation(grid_size, grid_size, cell_size, 50, 5, 100, 0.01);
    
    generateOfficeMap(simulation.known_grid.occupancy, cell_size, 0.15f, 0.9f);
    // TODO: Add holes
    // addHoles(simulation.known_grid.occupancy, cell_size, muHoleSize, sigmaHoleSize);
    // generateSensorX(...)


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
            std::this_thread::sleep_for(std::chrono::microseconds(1));
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
