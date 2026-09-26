<p align="center">
  <img src="https://img.shields.io/badge/MATLAB-Simulation-orange" alt="MATLAB">
  <img src="https://img.shields.io/badge/Robotics-Ball%20on%20Plate-blue" alt="Robotics">
  <img src="https://img.shields.io/badge/Control-Trajectory%20Tracking-green" alt="Control">
  <img src="https://img.shields.io/badge/Visualization-3D-purple" alt="3D Visualization">
</p>

<p align="center">
  <img src="./ball_on_plate_research_grade.gif" alt="Ball-on-plate simulation animation" width="600">
</p>

<h1 align="center">Ball-on-Plate MATLAB Simulation</h1>

<p align="center">Nonlinear ball dynamics, trajectory tracking, and four-stepper actuation with live 3D visualization.</p>

## Overview

A MATLAB simulation of a ball moving on a tilting plate. A PID-style tracking controller with acceleration feedforward generates tilt commands, while four simulated stepper actuators drive the platform.

- Nonlinear rolling dynamics, friction, disturbances, and sensor noise.
- Smooth reference trajectory and live tracking diagnostics.
- Actuator lag, speed limits, microstepping, and backlash.
- Automatic video and PNG exports to the `result` folder.

## Run

Open `Ball_On_Plate_Research_Grade_Simulation.m` in MATLAB and click **Run**, or enter:

```matlab
Ball_On_Plate_Research_Grade_Simulation
```

No Control System Toolbox is required. Adjust duration, time step, and video export in **User Options**.

## Simulation Results

**Final simulation view**

<p align="center">
  <img src="./result/simulation_final.png" alt="Final simulation view with live diagnostics" width="600">
</p>

**Tracking and actuator response**

<p align="center">
  <img src="./result/simulation_results.png" alt="Trajectory, tracking error, plate angles, and stepper response" width="900">
</p>

The plots show reference versus actual motion, position error, roll/pitch, and leg-1 command tracking. The Command Window reports RMS error, maximum error, mean torque, and maximum tilt.

Video exports as MP4, or AVI when MPEG-4 is unavailable. This is a simulation model; hardware performance has not been validated.
