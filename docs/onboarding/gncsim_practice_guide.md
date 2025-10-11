# GNCSIM Practice Guide

A hands-on guide for new team members to get started with GNCSIM simulations and verify their setup is working correctly.

**Prerequisites:** Complete the [git setup guide](git_onboarding_instructions.md) first to ensure you have GNCSIM cloned and can access the repository.

## Table of Contents

- [What is GNCSIM?](#what-is-gncsim)
- [Understanding the Master File Structure](#understanding-the-master-file-structure)
- [Running Your First Simulation](#running-your-first-simulation)
- [Understanding the Simulation Components](#understanding-the-simulation-components)
- [Verifying Your Setup](#verifying-your-setup)
- [Next Steps](#next-steps)

## Understanding the Master File Structure

The heart of GNCSIM is the "master file" - a MATLAB script that orchestrates an entire simulation. Let's examine the Jet Vanes Rocket (JVR) master file as our example.

### Master File Location
Navigate to: `src/jetVanes_MasterFile.m`

This file demonstrates the standard GNCSIM workflow:

### 1. **Initialization Phase** (Lines 4-18)
```matlab
% Initialize rocket parameters
[Doof, doof_params] = setupRocket();

% Initialize aerodynamic parameters
[AeroObj, aero_forces_moments] = setupAerodynamics();

% Initialize environment parameters
EnvironmentObj = setupEnvironment();

% Initialize flight parameters
FlightObj = setupFlight();

% Generate trim points
TrimPointsObj = generateTrimPoints(...);
```

### 2. **Flight Simulation Phase** (Lines 20-24)
```matlab
FlightObj.set_runClosedLoop(true); % Enable controlled flight
FlightObj.set_applyWind(true);
log = FlightObj.runFlight(Doof, EnvironmentObj, AeroObj, TrimPointsObj);
FlightObj.plot_sim_dashboard(log, Doof, EnvironmentObj);
```

### 3. **Monte Carlo Analysis Phase** (Lines 26-35)
```matlab
MonteCarloObj = setupMonteCarlo(EnvironmentObj, FlightObj, Doof);
logMonteCarlo = MonteCarloObj.runMonteCarlo(...);
[MonteCarloObj, logMonteCarlo] = evaluateMonteCarlo(MonteCarloObj, logMonteCarlo);
MonteCarloObj.plotMonteCarlo(logMonteCarlo);
```

## Running Your First Simulation

Now let's run the JVR simulation to verify everything works correctly.

### Step 1: Open MATLAB
1. Launch MATLAB on your computer
2. Navigate to your GNCSIM directory: `C:\Users\[your-username]\Documents\GitHub\GNCSIM\src`

### Step 2: Check Your Working Directory
In the MATLAB command window, verify you're in the correct location:
```matlab
pwd
```
You should see a path ending with `\GNCSIM\src`

### Step 3: Modify Monte Carlo Settings (Recommended)
Before running the simulation, reduce the number of Monte Carlo runs for faster execution:

1. Open `jetVanes_MasterFile.m` in MATLAB editor
2. Find line 427: `MonteCarloObj.set_numRuns(100);`
3. Change it to: `MonteCarloObj.set_numRuns(4);`
4. Save the file

### Step 4: Run the Master File
Execute the jet vanes master file:
```matlab
jetVanes_MasterFile
```

### What Should Happen
The simulation will run through several phases:

1. **Initialization**: MATLAB will set up all simulation objects
2. **Flight Simulation**: You'll see progress as the simulation runs
3. **Dashboard Plot**: After the flight simulation, one comprehensive dashboard window will appear with multiple subplots showing:
      - 3D trajectory
      - Velocity and acceleration profiles
      - Control system inputs
      - Vehicle attitude and orientation
4. **Monte Carlo**: 4 simulation runs with parameter variations (if you modified the setting)
5. **Monte Carlo Results**: Two additional plot windows will appear:
      - 3D trajectory plot showing all 4 Monte Carlo runs
      - Euler angle plot showing attitude variations across runs

**Expected Runtime**: ~ 2 minutes depending on your computer

## Understanding the Simulation Components

While the simulation runs, let's understand what each component does:

### Core Classes (Located in `src/`)

1. **Rocket.m** - Defines vehicle properties
      - Mass properties (wet/dry mass, moments of inertia)
      - Geometry (diameter, length, center of gravity)
      - Thrust curve data
      - Parachute characteristics

2. **Aerodynamics.m** - Handles aerodynamic forces and moments
      - Drag, lift, and side forces
      - Pitching, rolling, and yawing moments
      - Angle of attack dependencies

3. **Environment.m** - Models environmental conditions
      - Atmospheric properties vs altitude
      - Wind models
      - Launch site location

4. **Flight.m** - Controls simulation execution
      - Integration time steps
      - Simulation duration
      - Control system activation
      - Initial conditions

5. **TrimPoints.m** - Generates linearized control points
      - Linearizes the trajectory to use linear controllers
      - Computes control gains
      - Creates reference trajectories

6. **MonteCarlo.m** - Performs statistical analysis
         - Parameter uncertainty modeling
         - Performance evaluation

### Key Simulation Features

**Flight Profile**: The simulation uses OpenRocket-generated flight data (`Flight Profile/JVR_sim.csv`) as a reference trajectory.

**Control System**: The rocket uses a linear-quadratic regulator (LQR) to maintain trajectory following.

**Monte Carlo Analysis**: Tests system robustness by varying parameters like:

- Mass properties (~10% typical)
- Thrust curve variations
- Environmental conditions (wind)
- Initial state uncertainties

## Verifying Your Setup

After running the simulation, verify everything worked correctly:

### Success Indicators

1. **No Error Messages**: The simulation completed without MATLAB errors
2. **Three Plot Windows Total**:
      - **Flight Dashboard**: One comprehensive window with multiple subplots showing trajectory, velocities, control inputs, and attitude
      - **Monte Carlo 3D Trajectory**: Shows all 4 simulation runs overlaid in 3D space
      - **Monte Carlo Euler Angles**: Shows attitude angle variations across the 4 runs
3. **Console Output**: MATLAB displayed progress messages during execution

### Common Issues and Solutions

**"Cannot find file 'JVR_sim.csv'"**

- Solution: Ensure you're running from the `src/` directory
- Check that `Flight Profile/JVR_sim.csv` exists

**"Undefined function or variable"**

- Solution: Make sure all GNCSIM class files are in your MATLAB path
- Try running `addpath(genpath(pwd))` from the GNCSIM root directory

**Simulation crashes or produces unrealistic results**

- Check MATLAB version compatibility (R2023b or newer recommended)
- Verify all required toolboxes are installed:

      - Aerospace Toolbox
      - Control Systems Toolbox
      - Statistics and Machine Learning Toolbox
      - Navigation Toolbox
      - Parallel Computing Toolbox
      - Symbolic Math Toolbox

**Plots don't appear**

- Try `figure` command to bring windows to front
- Check if plots are minimized in taskbar

---

Congratulations! You've successfully run your first GNCSIM simulation.

## Getting Help

If you encounter issues:

1. **Check Documentation**: [GNCSIM GitHub Pages](https://github.gatech.edu/pages/gnc/GNCSIM)
2. **Ask Team Members**: Old members are familiar with common setup issues
3. **Review Git Guide**: Many problems stem from incorrect repository setup
4. **MATLAB Help**: Use `help [function_name]` for built-in MATLAB functions