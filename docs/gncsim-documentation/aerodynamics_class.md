# `Aerodynamics` Class Documentation

The `Aerodynamics` class encapsulates aerodynamic models for calculating aerodynamic forces and moments on a rocket in body coordinates. It is designed to be flexible: by assigning user-defined function handles, you can adapt it to different aerodynamic scenarios, from simple approximations to CFD-based data.

---

## Class Overview

```matlab
classdef Aerodynamics < handle
    ...
end
```

- **Inheritance**: Inherits from MATLAB’s built-in `handle` class, so instances are passed by reference.
- **Purpose**: Provides a modular interface for computing aerodynamic forces and moments based on the rocket’s current state.

---

## Properties

- **`FAx_func` (function handle)**  
  Function for computing aerodynamic force in the body x-direction.

- **`FAy_func` (function handle)**  
  Function for computing aerodynamic force in the body y-direction.

- **`FAz_func` (function handle)**  
  Function for computing aerodynamic force in the body z-direction.

- **`MAx_func` (function handle)**  
  Function for computing aerodynamic moment about the body x-axis.

- **`MAy_func` (function handle)**  
  Function for computing aerodynamic moment about the body y-axis.

- **`MAz_func` (function handle)**  
  Function for computing aerodynamic moment about the body z-axis.

- **Why Function Handles?**  
  Allows the user to assign arbitrary aerodynamic models. Enables flexible support for varying fidelity models.

---

## Methods

### Constructor

```matlab
function obj = Aerodynamics()
    ...
end
```

- **Description**:
  - Initializes a new `Aerodynamics` object.
  - Function handles are empty (`[]`) by default and must be set explicitly by the user.

---

### `set_FAx_func`

```matlab
function set_FAx_func(obj, FAx_function)
    ...
end
```

- **Description**: Assigns the function handle for body-frame force in x-direction.

---

### `set_FAy_func`

```matlab
function set_FAy_func(obj, FAy_function)
    ...
end
```

- **Description**: Assigns the function handle for force in y-direction.

---

### `set_FAz_func`

```matlab
function set_FAz_func(obj, FAz_function)
    ...
end
```

- **Description**: Assigns the function handle for force in z-direction.

---

### `set_MAx_func`

```matlab
function set_MAx_func(obj, MAx_function)
    ...
end
```

- **Description**: Assigns the function handle for moment about x-axis.

---

### `set_MAy_func`

```matlab
function set_MAy_func(obj, MAy_function)
    ...
end
```

- **Description**: Assigns the function handle for moment about y-axis.

---

### `set_MAz_func`

```matlab
function set_MAz_func(obj, MAz_function)
    ...
end
```

- **Description**: Assigns the function handle for moment about z-axis.

---

### `get_FAx_func`

```matlab
function FAx = get_FAx_func(obj)
    ...
end
```

- **Description**: Returns function handle for x-force.

---

### `get_FAy_func`

```matlab
function FAy = get_FAy_func(obj)
    ...
end
```

- **Description**: Returns function handle for y-force.

---

### `get_FAz_func`

```matlab
function FAz = get_FAz_func(obj)
    ...
end
```

- **Description**: Returns function handle for z-force.

---

### `get_MAx_func`

```matlab
function MAx = get_MAx_func(obj)
    ...
end
```

- **Description**: Returns function handle for x-moment.

---

### `get_MAy_func`

```matlab
function MAy = get_MAy_func(obj)
    ...
end
```

- **Description**: Returns function handle for y-moment.

---

### `get_MAz_func`

```matlab
function MAz = get_MAz_func(obj)
    ...
end
```

- **Description**: Returns function handle for z-moment.

---

### `calcAeroBodyFrame_JV`

```matlab
function [FAb, MAb] = calcAeroBodyFrame_JV(obj, u, v, w, p, r, rho, CG)
    ...
end
```

- **Inputs**:
  - `u`, `v`, `w` (m/s): Body-frame velocity components
  - `p`, `r` (rad/s): Angular velocities
  - `rho` (kg/m³): Air density
  - `CG` (m): Center of gravity from nose tip

- **Outputs**:
  - `FAb`: [FAx; FAy; FAz] – aerodynamic forces in body frame
  - `MAb`: [MAx; MAy; MAz] – aerodynamic moments in body frame

- **Description**:
  - Evaluates all force and moment function handles.
  - Returns total aerodynamic force and moment vectors.

**Example:**

```matlab
[FAb, MAb] = aeroObj.calcAeroBodyFrame_JV(u, v, w, p, r, rho, CG);
```

---

## Typical Usage Workflow

```matlab
% Instantiate
aeroObj = Aerodynamics();

% Assign force/moment models
aeroObj.set_FAx_func(@(u,v,w,rho,CG,p,r) ... );
aeroObj.set_FAy_func(@(u,v,w,rho,CG,p,r) ... );
aeroObj.set_FAz_func(@(u,v,w,rho,CG,p,r) ... );
aeroObj.set_MAx_func(@(u,v,w,rho,CG,p,r) ... );
aeroObj.set_MAy_func(@(u,v,w,rho,CG,p,r) ... );
aeroObj.set_MAz_func(@(u,v,w,rho,CG,p,r) ... );

% Use in simulation
[FAb, MAb] = aeroObj.calcAeroBodyFrame_JV(u, v, w, p, r, rho, CG);
```

---

## Conclusion

The `Aerodynamics` class provides a modular and extensible interface for computing aerodynamic effects in flight simulations. By using function handles for each axis, users can customize the aerodynamic response to their simulation fidelity—whether empirical, lookup-based, or CFD-driven—while maintaining consistent access patterns.
