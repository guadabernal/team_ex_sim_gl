#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>
#include <iostream>
#include <vector>
#include <cmath>  // For cosf() and sinf()

// Window size
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 800;

// GLFW error callback
void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error (" << error << "): " << description << std::endl;
}

struct RescueRobot {
    float x, y, theta;
    float vx, vy;
    float battery;
    int time_drop;
    std::vector<int> sensors;

    void meassure() { }
    void move() { }

    float sensor_co2() { return 0; }
    int sensor_aqi() { return 0; }
    float sensor_heat() { return 0; }
    std::vector<float> sensor_lidar() { return std::vector<float>(); }
};

struct Grid {
    int meters;  // Size of each cell in "meters" (used as a scaling factor)
    std::vector<std::vector<float>> co2;
    std::vector<std::vector<float>> heat;
    std::vector<std::vector<int>> ground_type;
    std::vector<std::vector<float>> incline;
    std::vector<std::vector<int>> occupancy;

    // Constructor: initializes all maps with 'rows' x 'cols' elements.
    Grid(int rows, int cols, int scale_m)
        : co2(rows, std::vector<float>(cols, 0.0f)),
        heat(rows, std::vector<float>(cols, 0.0f)),
        ground_type(rows, std::vector<int>(cols, 0)),
        incline(rows, std::vector<float>(cols, 0.0f)),
        occupancy(rows, std::vector<int>(cols, 0)),
        meters(scale_m)
    {}
};

void setSampleGrid(Grid& grid) {
    // Get the number of rows and columns from the occupancy grid.
    int rows = grid.occupancy.size();
    if (rows == 0) return;
    int cols = grid.occupancy[0].size();

    // 1. Fill the grid with ground (1)
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            grid.occupancy[y][x] = 1;  // Ground
        }
    }

    // 2. Create walls (0) along the border of the grid.
    // Top and bottom borders.
    for (int x = 0; x < cols; ++x) {
        grid.occupancy[0][x] = 0;            // Top wall
        grid.occupancy[rows - 1][x] = 0;       // Bottom wall
    }
    // Left and right borders.
    for (int y = 0; y < rows; ++y) {
        grid.occupancy[y][0] = 0;              // Left wall
        grid.occupancy[y][cols - 1] = 0;         // Right wall
    }

    // 3. Add a couple of holes (2) inside the ground.
    // Check if the grid is large enough to place holes without interfering with walls.
    if (rows > 4 && cols > 4) {
        grid.occupancy[2][2] = 2;               // Hole at (2, 2)
        grid.occupancy[rows / 2][cols / 2] = 2; // Hole at the center of the grid
    }
}

class Simulation {
public:
    Grid known_grid;
    Grid grid;

    Simulation()
        : known_grid(100, 100, 10),  // 100x100 cells, each cell is "10 meters" wide
        grid(100, 100, 10)
    {}
};

int main() {
    // Initialize GLFW
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Create OpenGL 1.0 window
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "OpenGL 1.0 + ImGui", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    // Initialize ImGui for OpenGL 1.0
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    // Create a Simulation instance and initialize the known occupancy grid.
    Simulation simulation;
    setSampleGrid(simulation.known_grid);

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();  // Handle input

        // (Optional) Start ImGui frame if needed
        // ImGui_ImplOpenGL2_NewFrame();
        // ImGui_ImplGlfw_NewFrame();
        // ImGui::NewFrame();

        // Clear screen
        glClear(GL_COLOR_BUFFER_BIT);

        // Set up an orthographic projection so our coordinates map to window pixels.
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        // ---- Render the occupancy grid ----

        // Get grid dimensions.
        int gridRows = simulation.known_grid.occupancy.size();
        int gridCols = simulation.known_grid.occupancy[0].size();
        // Compute a scale factor so the entire grid fits the window.
        // The total grid width (in "meters") is gridCols * simulation.known_grid.meters.
        float scaleFactor = float(WINDOW_WIDTH) / (gridCols * simulation.known_grid.meters);
        // Each cell will be drawn as a square with side length:
        float cellSize = simulation.known_grid.meters * scaleFactor;

        // Draw each cell as a quad with a color based on its occupancy value.
        for (int y = 0; y < gridRows; ++y) {
            for (int x = 0; x < gridCols; ++x) {
                int cell = simulation.known_grid.occupancy[y][x];
                // Set color based on occupancy value.
                switch (cell) {
                case 0: // Wall
                    glColor3f(1.0f, 0.0f, 0.0f);
                    break;
                case 1: // Ground
                    glColor3f(0.0f, 0.8f, 0.0f);
                    break;
                case 2: // Hole
                    glColor3f(0.0f, 0.0f, 0.8f);
                    break;
                default:
                    glColor3f(1.0f, 1.0f, 1.0f);
                    break;
                }
                // Compute the position of the current cell.
                float left = x * cellSize;
                float top = y * cellSize;
                float right = left + cellSize;
                float bottom = top + cellSize;

                // Draw the cell as a filled quad.
                glBegin(GL_QUADS);
                glVertex2f(left, top);
                glVertex2f(right, top);
                glVertex2f(right, bottom);
                glVertex2f(left, bottom);
                glEnd();
            }
        }
        // ---- End grid rendering ----

        // Restore the previous matrix state
        glPopMatrix(); // Modelview
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);

        // (Optional) Render ImGui on top if needed.
        // ImGui::Render();
        // ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
