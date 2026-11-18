# Project overview

## Scope

The course project, *Digital Control Systems — Discrete Time Sliding Mode Control Design for Unicycle Robot*, was developed in academic year 2024/2025. It studies stabilization of a unicycle mobile robot at the origin under actuator limits.

The main contribution is a discrete-time sliding-mode controller based on the chained form of the unicycle model. Two continuous-time alternatives are also implemented for comparison.

## Work packages

### Unicycle kinematics and chained form

The configuration is [x, y, theta] and the physical inputs are linear velocity v and angular velocity omega:

- dx/dt = v cos(theta)
- dy/dt = v sin(theta)
- dtheta/dt = omega

The report uses the transformation x1 = x, x2 = tan(theta), x3 = y, with u1 = v cos(theta) and u2 = omega sec^2(theta), to obtain chained form.

### Discrete-time sliding-mode control

The chained-form model is discretized using multirate input sampling. Sliding variables and equivalent control inputs are designed to drive the states to the origin in a finite number of steps. The analysis considers bounded inputs, actuator saturation, the domain of attraction, and a perturbed-unicycle case.

### Alternative controllers

The project also implements regulation through polar coordinates and dynamic feedback linearization using an extended unicycle state.

### Simulation results

The MATLAB scripts generate planar trajectories, state histories, and control-input plots. The report presents results for the three controllers and compares the approaches.

## Original material

The archive included a written report (Digital_Control_Systems.pdf), a presentation (SGAB.pdf), MATLAB Live Scripts, MATLAB function files, research papers, and digital-control textbooks. The Live Script code has been exported to .m files and arranged by model and controller.

The report and presentation remain in the original source archive and are not duplicated in this initial repository organization pass.