#define NOMINMAX

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

#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cstdlib>

// Jsoon File Format:
//    {
//      "experiment_conditions": {
//        "map_size": 20.0,
//        "cell_size" : 0.05,
//        "occupancy_map" : [...] ,
//        "people_locations" : [...] ,
//        "incline_map" : [...] ,
//        "total_runtime" : 123.45
//      },
//      "results": [
//        { ... simulation result 1 ... },
//        { ... simulation result 2 ... }
//      ]
//    }

template<typename T>
std::string vector2DToJson(const std::vector<std::vector<T>>& vec) {
    std::stringstream ss;
    ss << "[";
    bool firstRow = true;
    for (const auto& row : vec) {
        if (!firstRow) ss << ", ";
        ss << "[";
        bool firstElem = true;
        for (const auto& elem : row) {
            if (!firstElem) ss << ", ";
            ss << elem;
            firstElem = false;
        }
        ss << "]";
        firstRow = false;
    }
    ss << "]";
    return ss.str();
}

// Convert a vector of pairs to a JSON array string.
std::string pairVectorToJson(const std::vector<std::pair<float, float>>& vec) {
    std::stringstream ss;
    ss << "[";
    bool first = true;
    for (const auto& p : vec) {
        if (!first) ss << ", ";
        ss << "[" << p.first << ", " << p.second << "]";
        first = false;
    }
    ss << "]";
    return ss.str();
}

// This function constructs the JSON string for a single simulation result.
std::string constructSimResult(const Simulation& sim) {
    int totalCells = sim.grid.foundBy.size() * sim.grid.foundBy[0].size();
    std::stringstream rrMetricsSS;
    int totalCoveredByRR = 0;
    bool firstRR = true;
    for (const auto& robot : sim.rr) {
        if (!robot.spawned)
            continue;
        float runtime = robot.dead ? (robot.timeDeath - robot.spawnTime) : (sim.t - robot.spawnTime);
        int discoveredCount = 0;
        for (const auto& row : sim.grid.foundBy)
            for (int cell : row)
                if (cell == robot.id)
                    discoveredCount++;
        float coverage = (static_cast<float>(discoveredCount) / totalCells) * 100.0f;
        totalCoveredByRR += discoveredCount;
        if (!firstRR)
            rrMetricsSS << ", ";
        rrMetricsSS << "{\"id\": " << robot.id
            << ", \"start_x\": " << robot.x
            << ", \"start_y\": " << robot.y
            << ", \"start_theta\": " << robot.theta
            << ", \"runtime\": " << runtime
            << ", \"coverage\": " << coverage << "}";
        firstRR = false;
    }
    int vineCount = 0;
    for (const auto& row : sim.grid.foundBy)
        for (int cell : row)
            if (cell == -2)
                vineCount++;
    float vineCoverage = (static_cast<float>(vineCount) / totalCells) * 100.0f;
    float totalCoverage = (static_cast<float>(totalCoveredByRR) / totalCells) * 100.0f;

    std::stringstream ss;
    ss << "{\n";
    ss << "  \"active_robots\": {\"vr\": " << (sim.vrActive ? "true" : "false")
        << ", \"rr\": " << (sim.rrActive ? "true" : "false") << "},\n";
    ss << "  \"num_rr_deployed\": "
        << std::count_if(sim.rr.begin(), sim.rr.end(), [](const RescueRobot& r) { return r.spawned; }) << ",\n";
    ss << "  \"rescue_robot_metrics\": [" << rrMetricsSS.str() << "],\n";
    ss << "  \"vine_robot_coverage\": " << vineCoverage << ",\n";
    ss << "  \"total_coverage\": " << totalCoverage << ",\n";
    ss << "  \"final_occupancy_map\": " << vector2DToJson(sim.grid.occupancy) << ",\n";
    ss << "  \"final_heat_map\": " << vector2DToJson(sim.grid.heat) << "\n";
    ss << "}";
    return ss.str();
}

// Save simulation results into a JSON file that maintains a single experiment_conditions block
// at the top and an array of results.
void saveSimulationResults(const Simulation& sim, const std::string& filename) {
    std::string simResult = constructSimResult(sim);
    std::ifstream ifs(filename);
    bool fileExists = ifs.good();
    std::string fileContent;
    if (fileExists) {
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        fileContent = buffer.str();
        ifs.close();
    }

    if (!fileExists || fileContent.empty()) {
        // File does not exist. Write a new JSON with experiment_conditions and a results array.
        std::stringstream jsonSS;
        jsonSS << "{\n";
        jsonSS << "  \"experiment_conditions\": {\n";
        jsonSS << "    \"map_size\": " << sim.consts.totalSize << ",\n";
        jsonSS << "    \"cell_size\": " << sim.consts.cellSize << ",\n";
        jsonSS << "    \"occupancy_map\": " << vector2DToJson(sim.known_grid.occupancy) << ",\n";
        jsonSS << "    \"people_locations\": " << pairVectorToJson(sim.personPositions) << ",\n";
        jsonSS << "    \"incline_map\": " << vector2DToJson(sim.known_grid.height) << ",\n";
        jsonSS << "    \"total_runtime\": " << sim.t << "\n";
        jsonSS << "  },\n";
        jsonSS << "  \"results\": [\n";
        jsonSS << simResult << "\n";
        jsonSS << "  ]\n";
        jsonSS << "}\n";
        std::ofstream ofs(filename);
        ofs << jsonSS.str();
        ofs.close();
        std::cout << "Simulation results saved to " << filename << std::endl;
    }
    else {
        // File exists. We assume it has the structure with an experiment_conditions block and a "results" array.
        // We remove the trailing "]" and "}" (plus any trailing whitespace) and then append a comma and the new result.
        size_t pos = fileContent.rfind("]");
        if (pos == std::string::npos) {
            std::cerr << "Unexpected file format in " << filename << std::endl;
            return;
        }
        std::string newContent = fileContent.substr(0, pos);
        // Check if the results array already has an entry by looking for a "{" after the "results" key.
        size_t resultsPos = newContent.find("\"results\"");
        bool hasResult = newContent.find("{", resultsPos) != std::string::npos;
        if (hasResult)
            newContent += ",\n" + simResult + "\n";
        else
            newContent += "\n" + simResult + "\n";
        newContent += "]\n}";
        std::ofstream ofs(filename);
        ofs << newContent;
        ofs.close();
        std::cout << "Appended simulation result to " << filename << std::endl;
    }
}

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

bool isCandidateValid(float x, float y,
    const std::vector<std::vector<int>>& occ,
    float cellSize, float totalSize) {
    float distToBoundary = std::min({ x, totalSize - x, y, totalSize - y });
    if (distToBoundary > 2.0f)
        return false; // Too far from outer boundary.

    for (size_t r = 0; r < occ.size(); r++) {
        for (size_t c = 0; c < occ[r].size(); c++) {
            int cell = occ[r][c];
            if (cell == 0 || cell == 2) {
                float cellCenterX = (c + 0.5f) * cellSize;
                float cellCenterY = (r + 0.5f) * cellSize;
                float dx = x - cellCenterX;
                float dy = y - cellCenterY;
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist < 0.25f)
                    return false;
            }
        }
    }
    return true;
}

// ----- Helper to choose a heading (theta) that "points in" -----
// We determine the nearest outer boundary and then choose an angle (in degrees)
// from a specific interval:
//  - If near the bottom (y is minimum): allowed range [20, 160]° (points upward).
//  - If near the top (totalSize-y is minimum): allowed range [200, 340]° (points downward).
//  - If near the left (x is minimum): allowed range is defined as angles near 0°.
//    Here we choose from [-20, 70]° and then normalize to [0,360).
//  - If near the right (totalSize-x is minimum): allowed range [110, 250]° (points left).
float chooseTheta(float x, float y, float totalSize, std::mt19937& gen) {
    float leftDist = x;
    float rightDist = totalSize - x;
    float bottomDist = y;
    float topDist = totalSize - y;
    float minDist = std::min({ leftDist, rightDist, bottomDist, topDist });

    std::uniform_real_distribution<float> angleDist;
    float theta_deg = 0.0f;
    if (minDist == bottomDist) {
        // Candidate is nearest the bottom wall; inward normal is upward.
        // Allowed heading: between 20° and 160°.
        angleDist = std::uniform_real_distribution<float>(20.0f, 160.0f);
        theta_deg = angleDist(gen);
    }
    else if (minDist == topDist) {
        // Nearest the top; inward normal is downward.
        // Allowed heading: between 200° and 340°.
        angleDist = std::uniform_real_distribution<float>(200.0f, 340.0f);
        theta_deg = angleDist(gen);
    }
    else if (minDist == leftDist) {
        // Nearest the left wall; inward normal is to the right (0°).
        // To avoid too shallow an angle, choose from -20° to 70°.
        angleDist = std::uniform_real_distribution<float>(-20.0f, 70.0f);
        theta_deg = angleDist(gen);
        if (theta_deg < 0)
            theta_deg += 360.0f;
    }
    else { // rightDist is smallest
        // Nearest the right wall; inward normal is to the left (180°).
        // Allowed range: [110, 250]°.
        angleDist = std::uniform_real_distribution<float>(110.0f, 250.0f);
        theta_deg = angleDist(gen);
    }
    // Convert degrees to radians.
    return theta_deg * 3.14159265f / 180.0f;
}


int main() {
    const int n_simulations = 1;
    const float totalSize = 20.0f;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> posDist(0.0f, 20.0f);
    // We won't use a simple angleDist anymore for theta;
    // instead we'll use chooseTheta() based on candidate position.
    std::bernoulli_distribution coin(0.5);


    for (int simIndex = 0; simIndex < n_simulations; simIndex++) {
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

        SimConsts simConsts;
        simConsts.cellSize = 0.05f;
        simConsts.totalSize = 20.0f;
        simConsts.nRobots = 10;
        simConsts.muHoleSize = 0.5f;
        simConsts.sigmaHoleSize = 0.2f;
        simConsts.nHoles = 8;
        simConsts.nPeople = 4;
        simConsts.maxTime = 30 * 60.0f;
        simConsts.dt = 0.1f;
        simConsts.rrTime = 60.0f;

        // Create a temporary occupancy grid for candidate testing.
        int rows = simConsts.getGridRows();
        int cols = simConsts.getGridCols();
        std::vector<std::vector<int>> tempOcc(rows, std::vector<int>(cols, -1));
        generateOfficeMap(tempOcc, simConsts.cellSize, 0.15f, 0.9f);
        std::vector<Hole> holes = generateHolesList_custom1();
        updateOccupancyWithHoles(tempOcc, holes, simConsts.cellSize);

        // Select valid start positions.
        float valid_vr_x, valid_vr_y, valid_rr_x, valid_rr_y;
        do {
            valid_vr_x = posDist(gen);
            valid_vr_y = posDist(gen);
        } while (!isCandidateValid(valid_vr_x, valid_vr_y, tempOcc, simConsts.cellSize, 20.0f));
        do {
            valid_rr_x = posDist(gen);
            valid_rr_y = posDist(gen);
        } while (!isCandidateValid(valid_rr_x, valid_rr_y, tempOcc, simConsts.cellSize, 20.0f));

        // Choose heading angles based on position.
        float vr_angle = chooseTheta(valid_vr_x, valid_vr_y, 20.0f, gen);
        float rr_angle = chooseTheta(valid_rr_x, valid_rr_y, 20.0f, gen);

        // Assign positions and angles.
        simConsts.vrX = valid_vr_x;
        simConsts.vrY = valid_vr_y;
        simConsts.rrX = valid_rr_x;
        simConsts.rrY = valid_rr_y;
        simConsts.vrAngle = vr_angle;
        simConsts.rrAngle = rr_angle;

        // Create the Simulation instance.
        Simulation simulation(simConsts);

        // Initialize robots using public initialization functions.
        simulation.initializeRescueRobots(valid_rr_x, valid_rr_y, rr_angle);
        simulation.vrActive = true;
        simulation.initializeVineRobot(valid_vr_x, valid_vr_y, vr_angle);
        /*if (coin(gen)) {
            simulation.vrActive = true;
            simulation.initializeVineRobot(valid_vr_x, valid_vr_y, vr_angle);
        }
        else {
            simulation.vrActive = false;
        }*/
        simulation.rrActive = true;


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
                    //std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    std::this_thread::sleep_for(std::chrono::nanoseconds(500));
                    lastRenderTime = std::chrono::steady_clock::now();
                }
                //std::this_thread::sleep_for(std::chrono::nanoseconds(10));
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
                renderGradientMap(simulation, renderScaleFactor);
                //renderHeightMap(simulation, renderScaleFactor);
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
            if (!robot.spawned) {
                std::cout << "Robot " << robot.id << ": Not spawned" << std::endl;
            }
            else {
                // Compute runtime from the actual drop moment.
                float runtime = robot.dead ? (robot.timeDeath - robot.spawnTime)
                    : (simulation.t - robot.spawnTime);

                int discoveredCount = 0;
                for (const auto& row : simulation.grid.foundBy) {
                    for (int cell : row) {
                        if (cell == robot.id) discoveredCount++;
                    }
                }
                coveredCells += discoveredCount;
                float percentExplored = (static_cast<float>(discoveredCount) / totalCells) * 100.0f;

                std::cout << "Robot " << robot.id << ": Run time = " << runtime << " sec, "
                    << (robot.dead ? "Fell " : "Alive")
                    << ", % area exp = " << percentExplored << "%" << std::endl;
            }
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

        saveSimulationResults(simulation, "merged_res_1.json");
    }
    std::system("python ../scripts/plot_results.py");
    return 0;
}


int main_0() {
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
    simConsts.nPeople = 4;
    simConsts.maxTime = 60 * 60.0f;
    simConsts.dt = 0.1f;

    // --------------- Create Simulation instance --------------------
    Simulation simulation(simConsts);

    simulation.vrActive = true;
    simulation.rrActive = false;

    //simulation.initializeRescueRobots(1.0f, 1.0f, 0.0f);
    simulation.initializeVineRobot(1.0f, 1.0f, 0.0f);
    


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
                //std::this_thread::sleep_for(std::chrono::milliseconds(1));
                //std::this_thread::sleep_for(std::chrono::milliseconds(10));
                //std::this_thread::sleep_for(std::chrono::nanoseconds(500));
                lastRenderTime = std::chrono::steady_clock::now();
            }
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
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
            renderGradientMap(simulation, renderScaleFactor);
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
        renderGradientMap(simulation, renderScaleFactor);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        glfwSwapBuffers(finalWindow);
    }

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(finalWindow);
    glfwTerminate();

    return 0;
}