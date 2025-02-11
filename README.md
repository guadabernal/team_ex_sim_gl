# team_ex_sim_gl

INPUT
  (1) RR (#'s, Locations, sensors)  
  (2) VR Start and angle  
  (3) VR Start and angle and times for dropping RR's (each has sensor specs)  
  (4) whether we are running until dead or given a simmulation time limit  
  (5) initial known maps (occupancy, sensors, incline)  

EACH TIME STEP  
  (1) Move robots  
  (2) update known maps  

OUTPUT
  (1) each robot's total run time (if it died)  
  (2) each robot's % of area explored (each grid section can only be explored once)  

Map Info (each map has a real and known version)
  (1) Occupancy -> each cell can be (wall, ground type, hole)  
  (2) Sensor Grids (Heat, CO2, Gas, vision) (might be faster to only do the ones we have sensors for?)  
  (3) incline -> split big grid into subsections with different incline cost vals (like collapsed buildings)  

Need Functions
  (1) function that moves the RRs  
  (2) function that computes RRs cost of motion based on (ground type / incline / sensors)  
  (2) function that moves the Vine Robot (independently)  
  (3) function that checks for collisions => RR cannot collide with vine robot (unless in intelligence motion mode but not crucial)  
  (3) function that updates known maps, based on robot location & sensor type  

Constants File
  - RR size 120 mm  

