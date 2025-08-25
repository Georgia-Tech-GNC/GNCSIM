# `Flight` Class Documentation

The `Flight` class is responsible for executing rocket flight simulations over user-defined time spans. It supports both open-loop and closed-loop modes, optional wind disturbances, and the choice between nonlinear or linearized rocket dynamics.

---

## Class Overview

```matlab
classdef Flight < handle
    ...
end
```

- **Inheritance**: Inherits from MATLAB’s built-in `handle` class, so changes to object properties persist outside method calls.
- **Purpose**: Manages time-stepping, logging, and control toggles for a rocket simulation over a specified time horizon.

---

## Properties

All properties are accessed and validated through public setters to ensure consistency.

- **`tStep` (double)**  
  - **Description**: Base simulation time step (in seconds).  
  - **Recommendation**: Use `tStep < 0.01` for improved accuracy and stability.  
  - **Note**: This affects logging resolution, not necessarily internal solver steps.

- **`runClosedLoop` (boolean)**  
  - **Description**: Enables or disables control application.  
  - **Note**: If `false`, the rocket behaves as if no control authority exists.

- **`calcXDots` (boolean)**  
  - **Description**: If `true`, recomputes state derivatives and auxiliary variables after integration.  
  - **Why**: Useful for logging quantities not returned by the solver (e.g., aerodynamic moments).

- **`runLinearModel` (boolean)**  
  - **Description**: Simulates a linearized model alongside the nonlinear one.  
  - **Note**: Used to validate linear model accuracy.

- **`applyWind` (boolean)**  
  - **Description**: Toggles the application of wind disturbances.

- **`numStates` (integer)**  
  - **Description**: Size of the state vector.  
  - **Default**: 13 (position, velocity, quaternion, angular velocity).

- **`numInputs` (integer)**  
  - **Description**: Number of control inputs.  
  - **Default**: 3 (roll, pitch, yaw moments).

- **`localSimTime` (double)**  
  - **Description**: Duration (in seconds) for each simulation sub-interval.  
  - **Recommendation**: Often set to 1 second for linear validation.

- **`simTime` (double)**  
  - **Description**: Total simulation duration (in seconds).  
  - **Note**: Typically matches motor burn time.

- **`tSpan` (vector)**  
  - **Description**: Time vector from `0` to `simTime` with step `tStep`.

- **`gravity` (double)**  
  - **Description**: Gravitational acceleration (default `9.81 m/s²`).

- **`uSpan` (`[length(tSpan) × numInputs]` matrix)**  
  - **Description**: External inputs applied at each timestep.  
  - **Usage**: Can simulate known disturbances or test control responses.

- **`windParam` (varies)**  
  - **Description**: Wind model parameters.  
  - **Usage**: Assigned externally or from the environment model.

- **`odeOptions` (struct)**  
  - **Description**: MATLAB ODE solver options (e.g., tolerances).  
  - **Usage**: Use `odeset` to customize.

- **`initState` (`[numStates × 1]` vector)**  
  - **Description**: Initial state vector.  
  - **Default**: Stationary launchpad state.

- **`controlsTime` (double)**  
  - **Description**: Placeholder for control law update scheduling.

- **`odeSolverMaxTime` (double)**  
  - **Description**: Maximum real-world time allowed for solver execution.  
  - **Default**: `1000` seconds.

- **`odeSolverTerminationFlag` (boolean)**  
  - **Description**: Indicates if solver was terminated early.

---

## Methods

### Constructor

```matlab
function obj = Flight()
    % Creates a Flight object with default settings.
end
```

---

### Setters & Getters

Each property includes validated setter and corresponding getter methods.

**Example:**

```matlab
myFlight.set_tStep(0.005);
myFlight.set_simTime(10);
myFlight.set_runClosedLoop(true);

curr_tStep = myFlight.get_tStep();
curr_simTime = myFlight.get_simTime();
```

---

### `runFlight`

```matlab
function log = runFlight(obj, Rocket, Environment, Aerodynamics, ControlObj)
    ...
end
```

- **Inputs**:
  - `Rocket`: Physical properties and thrust model
  - `Environment`: Atmospheric and wind conditions
  - `Aerodynamics`: Aerodynamic model
  - `ControlObj`: Control law or linearization (e.g., `TrimPoints`)
- **Output**: `log` (struct containing simulation data)
- **Description**:
  - Integrates the state equations over `simTime`
  - Recomputes state derivatives if `calcXDots` is enabled
  - Optionally includes linear model propagation

---

### `plot_sim_dashboard`

```matlab
function [] = plot_sim_dashboard(obj, log, Rocket, Environment)
    ...
end
```

- **Description**:
  - Plots 3D trajectory, body rates, quaternion evolution, and control inputs.
  - Displays model comparisons if `runLinearModel` is enabled.

**Example:**

```matlab
logData = myFlight.runFlight(myRocket, myEnvironment, myAero, myControl);
myFlight.plot_sim_dashboard(logData, myRocket, myEnvironment);
```

---

### `stopSolver`

```matlab
function status = stopSolver(obj, ~, ~, flag, startTime, maxTime)
    ...
end
```

- **Purpose**: Halts the solver if wall-clock time exceeds `maxTime`.
- **Return**: `1` to terminate, `0` to continue

---

### `timestepper`

```matlab
function [traj_t, traj_x] = timestepper(obj, x0, dt, tspan, RocketObj, EnvironmentObj, AerodynamicsObj, ControlObj)
    ...
end
```

- **Description**: Custom Runge-Kutta 4 (RK4) routine.
- **Outputs**:
  - `traj_t`: Time vector
  - `traj_x`: State trajectory

---

## Typical Usage Workflow

```matlab
flightObj = Flight();

% Configure simulation
flightObj.set_tStep(0.005);
flightObj.set_simTime(10);
flightObj.set_runClosedLoop(true);
flightObj.set_calcXDots(true);
flightObj.set_applyWind(false);

% Set initial conditions
flightObj.set_initState([0 0 0 0 0 0 1 0 0 0 0 0 0]');
flightObj.set_tSpan();  % auto-generates time vector

% Run simulation
logData = flightObj.runFlight(myRocket, myEnvironment, myAero, myControlObj);

% Plot results
flightObj.plot_sim_dashboard(logData, myRocket, myEnvironment);
```

---

## Conclusion

The `Flight` class consolidates all core elements of rocket simulation:

- Time management and control toggling
- Open-loop and closed-loop dynamics
- Optional wind disturbances
- Optional linear model propagation
- Comprehensive logging and plotting

This structure makes it easy to iterate between early design evaluation and advanced control testing by simply adjusting properties like `runClosedLoop`, `applyWind`, and `calcXDots`.

