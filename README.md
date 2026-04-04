# Mobile Robot Navigation System 

This project is a MATLAB simulation of a mobile robot navigation system using the Robotics System Toolbox. It shows how a robot can move, sense its position, and navigate in a warehouse environment.

## Project Overview

The system is designed as a step-by-step pipeline. Each part represents an important function used in real robots:

- Robot movement (Forward Kinematics)
- Control system (PID)
- Localisation (Bayesian method)
- Environment mapping (Occupancy Grid)
- Path following (Pure Pursuit)
- Path planning (A* and Artificial Potential Field)

## Features

- Simulates robot motion using a differential-drive model  
- Uses PID control for accurate path tracking  
- Combines sensor data and odometry for better localisation  
- Creates a 2D map with obstacles  
- Compares A* and APF path planning methods  

## Tools Used

- MATLAB (R2023b or later)  
- Robotics System Toolbox  

## How to Run

1. Open MATLAB  
2. Run each file step by step:
   - CW2_forward_kinematics.m  
   - CW2_pid_tracking.m  
   - CW2_localisation.m  
   - CW2_pure_pursuit.m  
   - CW2_navigation_comparison.m  
3. Check the figures generated in each step  

## Project Structure

/code → MATLAB scripts  
/report → Coursework report  
/figures → Simulation results  

## Limitations

- Robot is simplified as a point  
- Environment is static (no moving obstacles)  
- Localisation is simplified  
- No full system integration  

## Future Work

- Combine A* and APF together  
- Use particle filter for localisation  
- Add moving obstacles  
- Implement SLAM  

## Author

Your Name  
