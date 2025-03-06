#define NOMINMAX

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>
#include <Windows.h>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include <vector>
#include <cmath>
#include <random>
#include <string>
#include <algorithm>

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include "simul.hpp"
#include "simul_render.hpp"
#include "rescue_robot.hpp"


//const int WINDOW_WIDTH = 1800;
//const int WINDOW_HEIGHT = 1200;

const int WINDOW_WIDTH = 1600;
const int WINDOW_HEIGHT = 1600;

const int numCols = 2;
const int numRows = 2;
const int colWidth = WINDOW_WIDTH / numCols;
const int rowHeight = WINDOW_HEIGHT / numRows;

void glfw_error_callback(int error, const char* description) { std::cerr << "GLFW Error (" << error << "): " << description << std::endl; }


// Json File Format:
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
std::string vectorToJson(const std::vector<float>& vec) {
    std::stringstream ss;
    ss << "[";
    bool first = true;
    for (float val : vec) {
        if (!first) ss << ", ";
        ss << val;
        first = false;
    }
    ss << "]";
    return ss.str();
}
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
    ss << "  \"active_robots\": {\"vr\": " << (sim.vrActive ? "true" : "false") << ", \"rr\": " << (sim.rrActive ? "true" : "false") << "},\n";
    ss << "  \"num_rr_deployed\": " << std::count_if(sim.rr.begin(), sim.rr.end(), [](const RescueRobot& r) { return r.spawned; }) << ",\n";
    ss << "  \"rescue_robot_metrics\": [" << rrMetricsSS.str() << "],\n";
    ss << "  \"vine_robot_coverage\": " << vineCoverage << ",\n";
    ss << "  \"total_coverage\": " << totalCoverage << ",\n";
    ss << "  \"delta_drop_time\": " << sim.delta_drop_time << ",\n";
    ss << "  \"coverage_over_time\": " << pairVectorToJson(sim.coverageHistory) << ",\n";
    ss << "  \"final_occupancy_map\": " << vector2DToJson(sim.grid.occupancy) << ",\n";
    ss << "  \"final_heat_map\": " << vector2DToJson(sim.grid.heat) << ",\n";
    ss << "  \"person_found_times\": " << vectorToJson(sim.personFoundTimes) << "\n";
    ss << "}";
    return ss.str();
}
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

bool isCandidateValid(float x, float y, const std::vector<std::vector<int>>& occ, float cellSize, float totalSize) {
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
        angleDist = std::uniform_real_distribution<float>(50.0f, 130.0f);
        theta_deg = angleDist(gen);
    }
    else if (minDist == topDist) {
        // Nearest the top; inward normal is downward.
        // Allowed heading: between 200° and 340°.
        angleDist = std::uniform_real_distribution<float>(230.0f, 310.0f);
        theta_deg = angleDist(gen);
    }
    else if (minDist == leftDist) {
        // Nearest the left wall; inward normal is to the right (0°).
        // To avoid too shallow an angle, choose from -20° to 70°.
        angleDist = std::uniform_real_distribution<float>(-40.0f, 40.0f);
        theta_deg = angleDist(gen);
        if (theta_deg < 0)
            theta_deg += 360.0f;
    }
    else { // rightDist is smallest
        // Nearest the right wall; inward normal is to the left (180°).
        // Allowed range: [110, 250]°.
        angleDist = std::uniform_real_distribution<float>(140.0f, 220.0f);
        theta_deg = angleDist(gen);
    }
    // Convert degrees to radians.
    return theta_deg * 3.14159265f / 180.0f;
}

Simulation createSimulation(std::mt19937& gen, std::uniform_real_distribution<float>& posDist, bool vrEnable, bool rrEnable, float rrTime, float timeLength)
{
    SimConsts simConsts;
    simConsts.cellSize = 0.05f;
    simConsts.totalSize = 20.0f;
    simConsts.nRobots = 15;
    simConsts.muHoleSize = 0.5f;
    simConsts.sigmaHoleSize = 0.2f;
    simConsts.nHoles = 8;
    simConsts.nPeople = 4;
    simConsts.maxTime = timeLength; // 45 * 60.0f;
    simConsts.dt = 0.2f; //0.05 for experiments
    simConsts.rrTime = rrTime;
    simConsts.vrEnable = vrEnable;
    simConsts.rrEnable = rrEnable;

    // Create a temporary occupancy grid for candidate testing.
    int rows = simConsts.getGridRows();
    int cols = simConsts.getGridCols();
    std::vector<std::vector<int>> tempOcc(rows, std::vector<int>(cols, -1));
    generateOfficeMap(tempOcc, simConsts.cellSize, 0.15f, 0.9f);
    std::vector<Hole> holes = generateHolesList_custom1();
    updateOccupancyWithHoles(tempOcc, holes, simConsts.cellSize);

    // Find valid start position for the vine robot.
    float valid_vr_x, valid_vr_y;
    do {
        valid_vr_x = posDist(gen);
        valid_vr_y = posDist(gen);
    } while (!isCandidateValid(valid_vr_x, valid_vr_y, tempOcc, simConsts.cellSize, simConsts.totalSize));

    // For rescue robots, if vrEnable is false, set their start to match the vine’s.
    float valid_rr_x, valid_rr_y;
    if (vrEnable) {
        do {
            valid_rr_x = posDist(gen);
            valid_rr_y = posDist(gen);
        } while (!isCandidateValid(valid_rr_x, valid_rr_y, tempOcc, simConsts.cellSize, simConsts.totalSize));
    }
    else {
        valid_rr_x = valid_vr_x;
        valid_rr_y = valid_vr_y;
    }

    // Choose heading angles.
    float vr_angle = chooseTheta(valid_vr_x, valid_vr_y, simConsts.totalSize, gen);
    // If vine is not used for rescue, use the vine’s angle.
    float rr_angle = vrEnable ? chooseTheta(valid_rr_x, valid_rr_y, simConsts.totalSize, gen) : vr_angle;

    // Assign positions and angles.
    simConsts.vrX = valid_vr_x;
    simConsts.vrY = valid_vr_y;
    simConsts.rrX = valid_rr_x;
    simConsts.rrY = valid_rr_y;
    simConsts.vrAngle = vr_angle;
    simConsts.rrAngle = rr_angle;

    // Create the Simulation instance and initialize the robots.
    Simulation simulation(simConsts);
    simulation.initializeRescueRobots(valid_rr_x, valid_rr_y, rr_angle, &gen);
    simulation.initializeVineRobot(valid_vr_x, valid_vr_y, vr_angle, &gen);

    return simulation;
}

float computeCoverage(const Simulation& sim) {
    int totalCells = sim.grid.foundBy.size() * sim.grid.foundBy[0].size();
    int coveredCells = 0;
    for (const auto& robot : sim.rr) {
        if (!robot.spawned)
            continue;
        int discoveredCount = 0;
        for (const auto& row : sim.grid.foundBy) {
            for (int cell : row) {
                if (cell == robot.id)
                    discoveredCount++;
            }
        }
        coveredCells += discoveredCount;
    }
    return (static_cast<float>(coveredCells) / totalCells) * 100.0f;
}

// Helper function to draw text with a background rectangle.
void DrawTextWithBackground(ImDrawList* draw_list, ImFont* font, float font_size,
    const ImVec2& pos, const char* text,
    ImU32 text_color, ImU32 bg_color,
    const ImVec2& padding = ImVec2(5, 5))
{
    ImVec2 text_size = font->CalcTextSizeA(font_size, FLT_MAX, 0.0f, text);
    ImVec2 rect_min = ImVec2(pos.x - padding.x, pos.y - padding.y);
    ImVec2 rect_max = ImVec2(pos.x + text_size.x + padding.x, pos.y + text_size.y + padding.y);
    draw_list->AddRectFilled(rect_min, rect_max, bg_color);
    draw_list->AddText(font, font_size, pos, text_color, text);
}


int main() {
    // Toggle to enable (or disable) the GUI mode.
    bool useGUI = true;
    std::string outputFilename = "fullSet_n20.json";
    bool activeVR = true;
    bool activeRR = true;

    /*std::random_device rd;
    std::mt19937 gen(rd());*/
    
    std::uniform_real_distribution<float> posDist(0.0f, 20.0f);

    if (useGUI) {
        /*unsigned seed = 1;
        std::mt19937 gen(seed);

        Simulation simulation = createSimulation(gen, posDist, activeVR, 60.0f, 2*60.0f);
        simulation.vrActive = activeVR;
        simulation.rrActive = activeRR;*/
        const float timeLength = 90 * 60.0f;
        unsigned seed = 40;
        // VINE + RESCUE ROBOTS - one per min ----------------------------
        std::mt19937 gen1(seed);
        std::mt19937 gen2(seed);
        std::mt19937 gen3(seed);
        std::mt19937 gen4(seed);

        Simulation simulation1 = createSimulation(gen1, posDist, true,true, 60.0f, timeLength);
        Simulation simulation2 = createSimulation(gen2, posDist, false, true, 5.0f, timeLength);
        Simulation simulation3 = createSimulation(gen3, posDist, true, true, 0.0f, timeLength);
        Simulation simulation4 = createSimulation(gen4, posDist, true, false, -1.0f, timeLength);
        // ----------------------------------------------------------------



        glfwSetErrorCallback(glfw_error_callback);
        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW\n";
            return -1;
        }
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


        float viewWidth = WINDOW_WIDTH / 3.0f;
        //float viewWidth = WINDOW_WIDTH / 2.0f;
        float renderScaleFactor = (float)colWidth / simulation1.consts.totalSize;

        std::mutex simMutex;
        std::atomic<bool> running(true);
        bool simulationEnded = false;

        std::thread simThread([&simulation1, &simulation2, &simulation3, &simulation4, &simMutex, &running, &simulationEnded]() {
            DWORD_PTR simAffinityMask = 0x4; // CPU 1
            HANDLE hSimThread = GetCurrentThread();
            DWORD_PTR previousMask = SetThreadAffinityMask(hSimThread, simAffinityMask);
        
            if (previousMask == 0) { std::cerr << "Failed to set affinity for simulation thread." << std::endl; }
            else { std::cout << "Simulation thread affinity set successfully." << std::endl; }

            auto lastRenderTime = std::chrono::steady_clock::now();
            bool done = false;
            while (running.load() && !done) {
                {
                    std::lock_guard<std::mutex> lock(simMutex);
                    done = simulation1.update();     // SIMULATION GETS UPDATED HERE <----
                    simulation2.update();
                    simulation3.update();
                    simulation4.update();
                    //simulation.HeatUpdate_forPlot();
                }
                if (done) simulationEnded = true;
                auto now = std::chrono::steady_clock::now();
                auto renderElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastRenderTime);

                if (renderElapsed > std::chrono::milliseconds(33)) {
                    //std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    std::this_thread::sleep_for(std::chrono::nanoseconds(100));
                    lastRenderTime = std::chrono::steady_clock::now();
                }
                std::this_thread::sleep_for(std::chrono::nanoseconds(100));
            }
            running.store(false);
        });


        // Main rendering loop.
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
            glClear(GL_COLOR_BUFFER_BIT);  
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            {
                std::lock_guard<std::mutex> lock(simMutex);
                
                //Incline map
                glViewport(0, rowHeight, colWidth, rowHeight);
                glMatrixMode(GL_PROJECTION);
                glPushMatrix();
                glLoadIdentity();
                glOrtho(0, colWidth, rowHeight, 0, -1, 1);
                glMatrixMode(GL_MODELVIEW);
                glPushMatrix();
                glLoadIdentity();
                renderHeightMap(simulation1, renderScaleFactor);
                renderHeatMapOnly(simulation1, renderScaleFactor);
                renderMeasurementGridAlpha(simulation1, renderScaleFactor);
                renderRobots(simulation1, renderScaleFactor);
                renderVineRobot(simulation1, renderScaleFactor);
                glPopMatrix();
                glMatrixMode(GL_PROJECTION);
                glPopMatrix();

                glViewport(colWidth, rowHeight, colWidth, rowHeight);
                glMatrixMode(GL_PROJECTION);
                glPushMatrix();
                glLoadIdentity();
                glOrtho(0, colWidth, rowHeight, 0, -1, 1);
                glMatrixMode(GL_MODELVIEW);
                glPushMatrix();
                glLoadIdentity();
                renderHeightMap(simulation2, renderScaleFactor);
                renderHeatMapOnly(simulation2, renderScaleFactor);
                renderMeasurementGridAlpha(simulation2, renderScaleFactor);
                renderRobots(simulation2, renderScaleFactor);
                renderVineRobot(simulation2, renderScaleFactor);
                glPopMatrix();
                glMatrixMode(GL_PROJECTION);
                glPopMatrix();

                glViewport(0, 0, colWidth, rowHeight);
                glMatrixMode(GL_PROJECTION);
                glPushMatrix();
                glLoadIdentity();
                glOrtho(0, colWidth, rowHeight, 0, -1, 1);
                glMatrixMode(GL_MODELVIEW);
                glPushMatrix();
                glLoadIdentity();
                renderHeightMap(simulation3, renderScaleFactor);
                renderHeatMapOnly(simulation3, renderScaleFactor);
                renderMeasurementGridAlpha(simulation3, renderScaleFactor);
                renderRobots(simulation3, renderScaleFactor);
                renderVineRobot(simulation3, renderScaleFactor);
                glPopMatrix();
                glMatrixMode(GL_PROJECTION);
                glPopMatrix();

                glViewport(colWidth, 0, colWidth, rowHeight);
                glMatrixMode(GL_PROJECTION);
                glPushMatrix();
                glLoadIdentity();
                glOrtho(0, colWidth, rowHeight, 0, -1, 1);
                glMatrixMode(GL_MODELVIEW);
                glPushMatrix();
                glLoadIdentity();
                renderHeightMap(simulation4, renderScaleFactor);
                renderHeatMapOnly(simulation4, renderScaleFactor);
                renderMeasurementGridAlpha(simulation4, renderScaleFactor);
                renderRobots(simulation4, renderScaleFactor);
                renderVineRobot(simulation4, renderScaleFactor);
                glPopMatrix();
                glMatrixMode(GL_PROJECTION);
                glPopMatrix();



                //// True heat map
                //glViewport(0, 0, colWidth, rowHeight);
                //glMatrixMode(GL_PROJECTION);
                //glPushMatrix();
                //glLoadIdentity();
                //glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
                //glMatrixMode(GL_MODELVIEW);
                //glPushMatrix();
                //glLoadIdentity();
                ////renderHeatMap(simulation, renderScaleFactor);
                //renderRobots(simulation, renderScaleFactor);
                //renderVineRobot(simulation, renderScaleFactor);
                //glPopMatrix();
                //glMatrixMode(GL_PROJECTION);
                //glPopMatrix();


                //// Right Top: discovered occupancy grid with robots (lidar view)
                //glViewport(colWidth, rowHeight, colWidth, rowHeight);
                //glMatrixMode(GL_PROJECTION);
                //glPushMatrix();
                //glLoadIdentity();
                //glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
                //glMatrixMode(GL_MODELVIEW);
                //glPushMatrix();
                //glLoadIdentity();
                //renderMeasurementGrid(simulation, renderScaleFactor);
                //renderRobots(simulation, renderScaleFactor);
                //renderVineRobot(simulation, renderScaleFactor);
                //glPopMatrix();
                //glMatrixMode(GL_PROJECTION);
                //glPopMatrix();

                //// discovered heat map from the robots heat sensors
                //glViewport(colWidth, 0, colWidth, rowHeight);
                //glMatrixMode(GL_PROJECTION);
                //glPushMatrix();
                //glLoadIdentity();
                //glOrtho(0, viewWidth, WINDOW_HEIGHT / 2, 0, -1, 1);
                //glMatrixMode(GL_MODELVIEW);
                //glPushMatrix();
                //glLoadIdentity();
                //renderDiscoveredHeatMap(simulation, renderScaleFactor);
                //renderRobots(simulation, renderScaleFactor);
                //renderVineRobot(simulation, renderScaleFactor);
                //glPopMatrix();
                //glMatrixMode(GL_PROJECTION);
                //glPopMatrix();



                ////results map
                //glViewport(2*colWidth, rowHeight, colWidth, rowHeight);
                //glMatrixMode(GL_PROJECTION);
                //glPushMatrix();
                //glLoadIdentity();
                //glOrtho(0, viewWidth, rowHeight, 0, -1, 1);
                //glMatrixMode(GL_MODELVIEW);
                //glPushMatrix();
                //glLoadIdentity();
                //renderFifthFigure(simulation, renderScaleFactor);
                //renderRobots(simulation, renderScaleFactor);
                //renderVineRobot(simulation, renderScaleFactor);
                //glPopMatrix();
                //glMatrixMode(GL_PROJECTION);
                //glPopMatrix();
            }

            // Start a new ImGui frame.
            ImGui_ImplOpenGL2_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            // Get ImGui's foreground draw list and current font.
            ImDrawList* draw_list = ImGui::GetForegroundDrawList();
            ImFont* font = ImGui::GetFont();
            float largeFontSize = font->FontSize * 2.0f;  // Increase font size as needed.
            char buf[64];

            sprintf(buf, "Time: %.2f s", simulation1.t); // assuming simulation1.t holds the current time
            // Adjust position as needed; here it's placed 210 pixels from the right edge.
            DrawTextWithBackground(draw_list, font, largeFontSize, ImVec2(WINDOW_WIDTH - 210, 10),
                buf, IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128));

            // Compute coverage for each simulation.
            float cov1 = computeCoverage(simulation1);
            float cov2 = computeCoverage(simulation2);
            float cov3 = computeCoverage(simulation3);
            float cov4 = computeCoverage(simulation4);

            // Draw coverage percentage in the top left of each viewport.
            // Adjust the positions as needed. Here we assume (0,0) is the top-left corner of the window.
            sprintf(buf, "Coverage: %.2f%%", cov1);
            DrawTextWithBackground(draw_list, font, largeFontSize, ImVec2(10, 10),
                buf, IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128));

            sprintf(buf, "Coverage: %.2f%%", cov2);
            DrawTextWithBackground(draw_list, font, largeFontSize, ImVec2(810, 10),
                buf, IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128));

            sprintf(buf, "Coverage: %.2f%%", cov3);
            DrawTextWithBackground(draw_list, font, largeFontSize, ImVec2(10, 810),
                buf, IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128));

            sprintf(buf, "Coverage: %.2f%%", cov4);
            DrawTextWithBackground(draw_list, font, largeFontSize, ImVec2(810, 810),
                buf, IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128));

            // Render the ImGui frame.
            ImGui::Render();
            ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window);
            if (simulationEnded) {
                std::cout << "Closing Simulation" << std::endl;
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
        }

        running.store(false);
        simThread.join();

        /*saveHeightMapToCSV(simulation, "heightmap.csv");
        saveOccupancyToCSV(simulation, "occupancy.csv");
        saveHeatMapToCSV(simulation, "heatmap.csv");
        std::system("python ../scripts/plot_height_map.py heightmap.csv occupancy.csv");
        std::system("python ../scripts/plot_heat_map.py heatmap.csv occupancy.csv");*/

        // After the simulation ends, capture the final fourth figure (assumed drawn in the viewport at (colWidth, 0, colWidth, rowHeight))
        {
            // Make sure all OpenGL commands have finished
            glFlush();
            glFinish();

            // Set the viewport to the region of the fourth figure.
            // (This should match the viewport used when drawing the discovered heat map.)
            glViewport(colWidth, 0, colWidth, rowHeight);

            // Define width and height of the viewport in pixels.
            int width = colWidth;
            int height = rowHeight;
            std::vector<unsigned char> imageData(width * height * 3);

            // Read pixels from the current viewport (using GL_RGB and unsigned byte format)
            glReadPixels(colWidth, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, imageData.data());

            // Save the image as a PPM file (a simple format that many scripts can easily read)
            std::ofstream ofs("fourth_figure.ppm", std::ios::binary);
            if (!ofs) {
                std::cerr << "Error: could not open fourth_figure.ppm for writing." << std::endl;
            }
            else {
                ofs << "P6\n" << width << " " << height << "\n255\n";
                ofs.write(reinterpret_cast<char*>(imageData.data()), imageData.size());
                ofs.close();
                std::cout << "Fourth figure saved to fourth_figure.ppm" << std::endl;
            }

            // Now call your Python script to reformat the figure.
            // (Assuming your Python script is called 'format_figure.py' and accepts the image filename as an argument.)
            int ret = system("python ../scripts/format_figure.py fig1_vin_rr_1min.ppm");
            if (ret != 0) {
                std::cerr << "Error calling the python script." << std::endl;
            }
        }


        ImGui_ImplOpenGL2_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window);

        // --------------- Output Results to Terminal -----------------------------

        int totalCells = simulation1.grid.foundBy.size() * simulation1.grid.foundBy[0].size();
        int coveredCells = 0;

        for (auto& robot : simulation1.rr) {
            float runtime = robot.dead ? (robot.timeDeath - robot.spawnTime) : (simulation1.t - robot.spawnTime);

            int discoveredCount = 0;
            for (const auto& row : simulation1.grid.foundBy) {
                for (int cell : row) {
                    if (cell == robot.id) discoveredCount++;
                }
            }
            coveredCells += discoveredCount;
            float percentExplored = (static_cast<float>(discoveredCount) / totalCells) * 100.0f;

            std::cout << "Robot " << robot.id << ": Run time = " << runtime << " sec, " << (robot.dead ? "Fell " : "Alive") << ", % area exp = " << percentExplored << "%" << std::endl;
        }

        int vineCount = 0;
        for (const auto& row : simulation1.grid.foundBy) {
            for (int cell : row) {
                if (cell == -2) vineCount++;
            }
        }
        float vinePercent = (static_cast<float>(vineCount) / totalCells) * 100.0f;
        float totalCoveragePercent = (static_cast<float>(coveredCells) / totalCells) * 100.0f;

        std::cout << "Vine Robot discovered " << vinePercent << "% of the area." << std::endl;
        std::cout << "Total area coverage by rescue robots: " << totalCoveragePercent << "%" << std::endl;

    }
    else {
        // ----- Batch (Non-GUI) Mode -----
        const int n_simulations = 1;
        const float timeLength = 90 * 60.0f;

        //for (int simIndex = 0; simIndex < n_simulations; simIndex++) {
        //    unsigned seed = simIndex;
        //    
        //    // VINE ROBOT ONLY
        //    std::mt19937 gen(seed);
        //    Simulation sim1 = createSimulation(gen, posDist, true, -1.0f, timeLength);
        //    sim1.vrActive = true;
        //    sim1.rrActive = false;
        //    while (!sim1.update()) {}
        //    saveSimulationResults(sim1, outputFilename);

        //    // RESCUE ROBOTS ONLY - one per minute
        //    std::mt19937 gen2(seed);
        //    Simulation sim2 = createSimulation(gen2, posDist, true, 60.0f, timeLength);
        //    sim2.vrActive = false;
        //    sim2.rrActive = true;
        //    while (!sim2.update()) {}
        //    saveSimulationResults(sim2, outputFilename);

        //    // RESCUE ROBOTS ONLY - all at once
        //    std::mt19937 gen3(seed);
        //    Simulation sim3 = createSimulation(gen3, posDist, true, 5.0f, timeLength);
        //    sim3.vrActive = false;
        //    sim3.rrActive = true;
        //    while (!sim3.update()) {}
        //    saveSimulationResults(sim3, outputFilename);

        //    // VINE + RESCUE ROBOTS - one per min
        //    std::mt19937 gen4(seed);
        //    Simulation sim4 = createSimulation(gen4, posDist, true, 60.0f, timeLength);
        //    sim4.vrActive = true;
        //    sim4.rrActive = true;
        //    while (!sim4.update()) {}
        //    saveSimulationResults(sim4, outputFilename);


        //    // VINE + RESCUE ROBOTS - at tip on retraction (run separate)
        //    std::mt19937 gen5(seed);
        //    Simulation sim5 = createSimulation(gen5, posDist, true, 0.0f, timeLength);
        //    sim5.vrActive = true;
        //    sim5.rrActive = true;
        //    while (!sim5.update()) {}
        //    saveSimulationResults(sim5, outputFilename);



        //    std::cout << " =================== DONE " << simIndex << " =================== " << std::endl;
        //
        //}
         std::string cmd = "python ../scripts/plot_results.py " + outputFilename;
         std::system(cmd.c_str());
    }

    return 0;
}