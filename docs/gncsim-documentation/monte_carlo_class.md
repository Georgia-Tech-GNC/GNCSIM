# `MonteCarlo` Class Documentation

The `MonteCarlo` class is designed to run Monte Carlo simulations for rocket flight. It uses Latin Hypercube Sampling (LHS) to generate various initial conditions and parameters that are tested for performance and constraint satisfaction.

---

## Class Overview

```matlab
classdef MonteCarlo < handle
    ...
end
```

- **Inheritance**: Inherits from MATLAB’s built-in `handle` class, meaning changes to object properties inside methods persist outside the methods.
- **Purpose**: Automate the setup, execution, and evaluation of multiple flight simulations under different parameter draws.

---

## Properties

Although many of these properties are private, they are configured using public setter methods.

- **`runGaussianLHS` (boolean)**  
  Indicates whether Gaussian (normal) distributions should be used for LHS.

- **`runUniformLHS` (boolean)**  
  Indicates whether uniform distributions should be used for LHS.

- **`runCustomLHS` (boolean)**  
  Indicates whether a custom (pre-generated) LHS matrix will be provided.

- **`numRuns` (integer)**  
  Number of Monte Carlo runs to generate and simulate.

- **`odeSolverMaxTime` (numeric)**  
  Maximum wall-clock time allowed for each simulation.

- **`lhsSamples` (matrix, NxM)**  
  The Latin Hypercube Sample matrix: `N = numRuns`, `M = number of parameters`.

- **`evaluate_checkTermination` (boolean)**  
  Flags simulations that terminate early as failed runs.

- **`evaluate_checkRoll` (boolean)**  
  Enables evaluation of roll angle constraints.

- **`evaluate_checkPitch` (boolean)**  
  Enables evaluation of pitch angle constraints.

- **`evaluate_checkYaw` (boolean)**  
  Enables evaluation of yaw angle constraints.

- **`evaluate_checkAoA` (boolean)**  
  Enables evaluation of angle-of-attack constraints.

- **`evaluate_rollRange` (cell)**  
  Acceptable roll angle ranges over specific time intervals.

- **`evaluate_pitchRange` (cell)**  
  Acceptable pitch angle ranges over time.

- **`evaluate_yawRange` (cell)**  
  Acceptable yaw angle ranges over time.

- **`evaluate_AoARange` (cell)**  
  Acceptable AoA ranges over time.

- **`wetMass_3sigma` (numeric)**  
  3σ deviation for rocket wet mass.

- **`dryMass_3sigma` (numeric)**  
  3σ deviation for rocket dry mass.

- **`wetRollMOI_3sigma`, `wetPitchMOI_3sigma`, `wetYawMOI_3sigma` (numeric)**  
  3σ deviation for moments of inertia in the wet state.

- **`dryRollMOI_3sigma`, `dryPitchMOI_3sigma`, `dryYawMOI_3sigma` (numeric)**  
  3σ deviation for MOIs in the dry state.

- **`initialCG_3sigma`, `finalCG_3sigma` (numeric)**  
  3σ deviation for center of gravity (initial and final).

- **`burnTime_3sigma` (numeric)**  
  3σ deviation for motor burn time.

- **`thrustCurve_3sigma` (numeric)**  
  3σ deviation for thrust curve data.

---

## Methods

### **Constructor**

  ```matlab
  function obj = MonteCarlo()
  end
  ```

  - **Description**: Instantiates a MonteCarlo object with default property values.
  - **Usage**:

    ```matlab
    mc = MonteCarlo();
    ```

---

### **Setters**

These methods perform validation on inputs before assigning values to the object’s private properties.

- `set_numRuns(obj, value)`  
    Ensures value is a nonnegative integer.

- `set_runGaussianLHS(obj, value)`  
Enables or disables Gaussian LHS.

- `set_runCustomLHS(obj, value)`  
Enables use of custom LHS matrix.

- `set_runUniformLHS(obj, value)`  
Enables or disables uniform LHS.

- `set_wetMass_3sigma(obj, value)`, `set_dryMass_3sigma(obj, value)`, etc.  
Assigns 3σ deviations for mass, MOI, CG, etc.

- `set_thrustCurve_3sigma(obj, value)`  
Sets the 3σ deviation for the thrust curve.

- `set_lhsSamples(obj, value)`  
Assigns the user-provided LHS matrix.

- `set_odeSolverMaxTime(obj, value)`  
Assigns the max solver time.

- `set_evaluate_checkTermination(obj, value)`, `set_evaluate_checkRoll(obj, value)`, etc.  
Used to control which flight constraints get evaluated.

- `set_evaluate_rollRange(obj, value)`, `set_evaluate_pitchRange(obj, value)`, etc.  
Assigns time intervals and acceptable angle ranges (as cell arrays).

---

### **Getters**

- `get_lhsSamples(obj)`  
    Returns the `lhsSamples` matrix. Throws an error if empty.

    ```matlab
    function lhsSamples = get_lhsSamples(obj)
        ...
    end
    ```

---

### `generateLhsSamples`

```matlab
function generateLhsSamples(obj, RocketObj)
    ...
end
```

- **Inputs**:
  - `RocketObj`: Rocket object containing nominal parameters
- **Description**:
  - Generates the LHS matrix using one of three modes:
    - Gaussian: Uses 3σ deviations and `lhsnorm`
    - Uniform: Samples [0, 1] range
    - Custom: Requires `lhsSamples` to be preloaded

---

### `runMonteCarlo`

```matlab
function logMonteCarlo = runMonteCarlo(obj, FlightObj, RocketObj, EnvironmentObj, AerodynamicsObj, TrimPointObj)
    ...
end
```

- **Inputs**:
  - `FlightObj`: Flight simulation object
  - `RocketObj`: Rocket properties object
  - `EnvironmentObj`: Environment object (e.g. wind)
  - `AerodynamicsObj`: Aerodynamics object
  - `TrimPointObj`: Trim points or control law info
- **Output**:
  - `logMonteCarlo`: Array of structs containing results for each simulation
- **Description**:
  - Runs all simulations in parallel (`parfor`)
  - Injects sample parameters into each rocket instance
  - Uses `FlightObj.runFlight(...)` to simulate

---

### `evaluateMonteCarlo`

```matlab
function [logMonteCarlo, performance] = evaluateMonteCarlo(obj, logMonteCarlo)
    ...
end
```

- **Inputs**:
  - `logMonteCarlo`: Simulation logs to evaluate
- **Outputs**:
  - `logMonteCarlo`: Updated with pass/fail flags
  - `performance`: Optional summary statistics
- **Description**:
  - Compares run results against:
    - ODE termination flags
    - Roll, pitch, yaw constraints
    - AoA constraints

---

### `saveToCSV`

```matlab
function saveToCSV(obj, filename)
    ...
end
```

- **Inputs**:
  - `filename`: Path to save `.csv` file
- **Description**:
  - Exports 3σ parameters and config to CSV for recordkeeping

---

### `loadFromCSV`

```matlab
function obj = loadFromCSV(obj, filename)
    ...
end
```

- **Inputs**:
  - `filename`: Path to a `.csv` file
- **Description**:
  - Imports saved parameters and updates object properties

---

### `splitLHS`

```matlab
function splittedLHS = splitLHS(obj, nParts, outFolder)
    ...
end
```

- **Inputs**:
  - `nParts`: Number of chunks to divide LHS into
  - `outFolder`: Directory to save chunks as `.mat` files
- **Output**:
  - `splittedLHS`: Cell array of LHS partitions
- **Description**:
  - Splits the `lhsSamples` matrix for distributed simulation
  - Requires `lhsSamples` to be non-empty and divisible

---

### `plotMonteCarlo` (Static)

```matlab
function plotMonteCarlo(logMonteCarlo)
    ...
end
```

- **Inputs**:
  - `logMonteCarlo`: Monte Carlo results to visualize
- **Description**:
  - Plots:
    - 3D final trajectory distribution
    - Yaw angle evolution with ±1σ and ±3σ bounds
    - AoA and other quantities (optional)

---

## Typical Usage Workflow

```matlab
% Instantiate
mc = MonteCarlo();

% Set parameters
mc.set_numRuns(1000);
mc.set_runGaussianLHS(true);
mc.set_odeSolverMaxTime(50);

% Generate samples
mc.generateLhsSamples(myRocket);

% OR load custom LHS
customLHS = ...;
mc.set_lhsSamples(customLHS);
mc.set_runCustomLHS(true);

% Run simulation
results = mc.runMonteCarlo(myFlight, myRocket, myEnv, myAero, myTrim);

% Evaluate results
mc.set_evaluate_checkRoll(true);
results = mc.evaluateMonteCarlo(results);

% Plot
MonteCarlo.plotMonteCarlo(results);

% Save/load
mc.saveToCSV('params.csv');
mc.loadFromCSV('params.csv');
```

---

## Conclusion

The `MonteCarlo` class provides a systematic way to:

- Define parameter uncertainty (mass, CG, MOI, thrust, etc.)
- Run parallel simulations for statistical robustness
- Evaluate whether simulations satisfy flight constraints
- Organize and export results for deeper analysis

It is essential for robust flight design, verification, and risk-aware performance evaluation.