# Mobile Robot Navigation System 

This project presents a MATLAB-based simulation of a mobile robot navigation system using the Robotics System Toolbox. It demonstrates how a robot can move, sense its position, and navigate safely in a warehouse environment.

## Project Overview

The system is designed as a step-by-step pipeline, where each part represents an important function used in real robotic systems:

- Robot motion using forward kinematics  
- Path tracking using PID control  
- Position estimation using Bayesian localisation  
- Environment representation using occupancy grid mapping  
- Path following using Pure Pursuit  
- Path planning using A* and Artificial Potential Field (APF)  

## Key Features

- Differential-drive robot motion model  
- Accurate trajectory tracking with PID control  
- Improved localisation using sensor fusion  
- 2D environment with obstacles  
- Comparison of A* (optimal) and APF (reactive) navigation methods  

## Tools Used

- MATLAB (R2023b or later)  
- Robotics System Toolbox  

## How to Run

1. Open MATLAB  
2. Run the scripts step by step:
   - CW2_forward_kinematics.m  
   - CW2_pid_tracking.m  
   - CW2_localisation.m  
   - CW2_pure_pursuit.m  
   - CW2_navigation_comparison.m  
3. Observe the figures generated at each stage  

## Limitations

- Robot is simplified as a point  
- Static environment (no moving obstacles)  
- Simplified localisation model  
- No full system integration  

## Future Improvements

- Combine A* and APF for better navigation  
- Use particle filter for more accurate localisation  
- Add dynamic (moving) obstacles  
- Implement SLAM (mapping and navigation together)  

## Author

Hasinu Ravishka
