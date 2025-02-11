# team_ex_sim_gl

## Cloning the Repository
To properly clone this repository, including all required submodules, use the following command:

```sh
git clone --recurse-submodules https://github.com/guadabernal/team_ex_sim_gl.git
```

If you have already cloned the repository without submodules, you can initialize them manually:

```sh
cd team_ex_sim_gl
git submodule update --init --recursive
```

## Building the Project
1. Ensure you have CMake installed.
2. Create the build directory and generate the Visual Studio project files:

```sh
mkdir build
cd build
cmake ..
```

### Option 1: Build and Run Using CMake
3. Build the project:

```sh
cmake --build .
```

4. Run the executable:

```sh
./team_ex_sim_gl
```

### Option 2: Build and Run Using Visual Studio
3. Open the generated Visual Studio solution file:

```sh
start team_ex_sim_gl.sln
```

4. Build the project inside Visual Studio.
5. Set `team_ex_sim_gl` as the startup project.
6. Click **Run** to execute the application.


# Requirements:
### INPUT
    (1) RR (each => Location, time dropped, sensors)
    (2) VR Start and angle
    (3) VR Start and angle and RR's (each => times dropped, sensors)
    (4) whether we are running until dead or given a simmulation time limit
    (5) initial known maps (occupancy, sensors, incline)

### EACH TIME STEP
    (1) Move robots
    (2) update known maps

### OUTPUT
    (1) each robot's total run time (if it died) 
    (2) each robot's % of area explored (each grid section can only be explored once)
    (3) final known maps

### Map Info: (each map has a real and known version)
    (1) Occupancy -> each cell can be (wall, ground type, hole)
    (2) Sensor Grids (Heat, CO2, Gas, vision) (might be faster to only do the ones we have sensors for?)
    (3) incline -> split big grid into subsections with different incline cost vals (like collapsed buildings)

### Need Functions:
    (1) function that moves the RRs 
    (2) function that computes RRs cost of motion based on (ground type / incline / sensors)
    (2) function that moves the Vine Robot (independently)
    (3) function that checks for collisions => RR cannot collide with vine robot (unless in intelligence motion mode but not crucial)
    (3) function that updates known maps, based on robot location & sensor type

### Constants File:
    RR size 120 mm
