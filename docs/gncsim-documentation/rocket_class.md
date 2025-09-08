# `Rocket` Class Documentation

The `Rocket` class represents a physical rocket with defined geometry, mass distribution, and motor parameters. It also provides methods to retrieve these parameters and compute thrust at any given time during the motor burn.

---

## Class Overview

```matlab
classdef Rocket < handle
    ...
end
```

- **Inheritance**: Inherits from MATLAB’s built-in `handle` class, meaning it is passed by reference.
- **Purpose**: Consolidate rocket-specific properties (mass, inertia, CG location, motor data) into one class. Users typically do not swap motors across different instances; the class assumes a single motor-rocket pairing.

---

## Properties

- **`wetMass` (double, kg)**  
  Mass of the rocket with a fully loaded motor. Used to compute total mass at liftoff.

- **`dryMass` (double, kg)**  
  Mass of the rocket after all propellant is burned.

- **`wetMOI` (3×3 matrix, kg·m²)**  
  Inertia tensor when rocket is fully fueled. Must be symmetric and positive definite.

- **`dryMOI` (3×3 matrix, kg·m²)**  
  Inertia tensor after propellant burnout. Must be square with positive diagonal.

- **`initialCG` (double, m)**  
  Distance from nose tip to center of gravity when fully loaded.

- **`finalCG` (double, m)**  
  Distance from nose tip to CG after burnout.

- **`CP` (double, m)**  
  Distance from nose tip to center of pressure. Typically fixed for a given geometry.

- **`length` (double, m)**  
  Physical length of the rocket, nose to tail.

- **`diameter` (double, m)**  
  Diameter of the main rocket body.

- **`radius` (double, m)**  
  Half the diameter. Used in geometry-based calculations.

- **`burnTime` (double, s)**  
  Time from ignition to motor burnout. Used for thrust and mass interpolation.

- **`thrustCurve` (n×2 matrix)**  
  `[time, thrust]` data points defining the motor’s thrust profile.

- **`mainChuteCd` (double)**  
  Drag coefficient of the main parachute.

- **`mainChuteArea` (double, m²)**  
  Reference area of the main parachute.

- **`drogueChuteCd` (double)**  
  Drag coefficient of the drogue parachute.

- **`drogueChuteArea` (double, m²)**  
  Reference area of the drogue parachute.

---

## Methods

### Constructor

```matlab
function obj = Rocket()
    % Constructs a new Rocket object with default (uninitialized) properties.
end
```

- **Description**: Instantiates a new `Rocket` object. Properties must be set with setter methods before use.

---

### Setters and Getters

Each property has a corresponding setter and getter that validates input (e.g., positive values, matrix shape).

**Examples:**

```matlab
myRocket.set_wetMass(30.0);
myRocket.set_dryMass(20.0);
myRocket.set_wetMOI([2,0,0; 0,3,0; 0,0,3]);
myRocket.set_dryMOI([1,0,0; 0,1.5,0; 0,0,1.5]);
...
disp(myRocket.get_wetMass());
```

- Setters enforce numeric types, matrix size, and nonnegativity.
- Getters return current property values.

---

### `getThrust`

```matlab
function currentThrust = getThrust(obj, t)
    ...
end
```

- **Inputs**:
  - `t` (double): Time since ignition in seconds
- **Output**:
  - `currentThrust` (double): Thrust in Newtons at time `t`
- **Description**:
  - Returns 0 if `t > burnTime`
  - If `t` matches a value in `thrustCurve`, return it directly
  - Otherwise, interpolates linearly between the surrounding points

**Usage:**

```matlab
thrustVal = myRocket.getThrust(2.5); % Thrust at 2.5 seconds
```

---

## Typical Usage Workflow

```matlab
% Instantiate
myRocket = Rocket();

% Set properties
myRocket.set_wetMass(30.0);
myRocket.set_dryMass(20.0);
myRocket.set_wetMOI([2,0,0; 0,3,0; 0,0,3]);
myRocket.set_dryMOI([1,0,0; 0,1.5,0; 0,0,1.5]);
myRocket.set_initialCG(1.2);
myRocket.set_finalCG(1.5);
myRocket.set_length(2.0);
myRocket.set_diameter(0.15);
myRocket.set_thrustCurve(thrustData); % Nx2 [time, thrust]
myRocket.set_burnTime(5.0);

% Retrieve parameters
disp(myRocket.get_length());
F = myRocket.getThrust(3.0);
fprintf("Thrust at 3.0 s: %.2f N\n", F);
```

---

## Conclusion

The `Rocket` class centralizes all physical and motor-related parameters of a rocket. It defines:

- Wet/dry mass and inertia tensors
- Center of gravity and pressure locations
- Geometry and parachute specs
- Thrust curve lookup via `getThrust()`

This class provides clean access to flight-critical parameters, which can be used downstream in simulation classes like `Flight` or `MonteCarlo`, ensuring consistency and accuracy across the modeling workflow.
