# `TrimPoints` Class Documentation

The `TrimPoints` class handles the generation, organization, and storage of “trim points” – specific rocket states (plus controls) that serve as initial conditions for flight simulations or linear control analysis. It reads flight data from a CSV file, constructs baseline states for specific times, applies variations, and runs trimming procedures.

---

## Class Overview

```matlab
classdef TrimPoints < handle
    ...
end
```

- **Inheritance**: Inherits from MATLAB’s built-in `handle` class, so changes to properties persist when passed around.
- **Purpose**:
  - Reads and stores flight data from a CSV.
  - Builds multiple `TrimBatch` objects, each representing variations around a base state at a given time.
  - Performs trimming to solve for near-steady or desired conditions.
  - Exports resulting trim points to CSV or header files.

---

## Properties

- **`file_path` (string)**  
  Path to the CSV flight data file.

- **`flight_data` (table)**  
  Raw data read from the CSV, includes headers and text.

- **`num_rows`, `num_cols` (integers)**  
  Size of `flight_data`.

- **`flight_data_num` (numeric matrix)**  
  Cleaned numeric data (NaN rows removed).

- **`batches` (array of `TrimBatch` objects)**  
  Each batch represents a specific time's base state plus variations.

- **`num_batches` (integer)**  
  Number of batches.

- **`trim_points` (struct array)**  
  Combined list of generated (untrimmed) trim points.

- **`trimmed_points` (struct array)**  
  Trim points with optimized states and associated metadata (e.g., K, A).

- **`column_positions_csv` (array)**  
  Indices for variables like time, thrust, velocity.

- **`state_dict` (`containers.Map`)**  
  Maps string state names to column indices.

- **`reference_states` (struct array)**  
  Reference states for control, matched to specific times.

- **`controller_ID` (constant, string)**  
  Default controller name (typically 'LQR').

- **`state_names` (constant cell array)**  
  Ordered list of all possible state names.

- **`state_index_con` (constant array)**  
  Indices corresponding to `state_names`.

---

## Methods

### `generate_trim_batches`

```matlab
function generate_trim_batches(obj, time_arr)
    ...
end
```

- **Inputs**: `time_arr` – vector of times (seconds)
- **Description**: Creates one `TrimBatch` per time and initializes base states.

---

### `add_u_variations`

```matlab
function add_u_variations(obj, ...)
    ...
end
```

- **Description**: Adds velocity (u) variations to each batch around nominal values.

---

### `add_custom_variations`

```matlab
function add_custom_variations(obj, batch_indexes, name, var, data, include_base_state)
    ...
end
```

- **Description**: Allows the user to add custom state/control variations to selected batches.

---

### `generate_trim_points`

```matlab
function generate_trim_points(obj, rocket, env, flight, aero)
    ...
end
```

- **Description**: Generates trim points and performs trimming optimization for each one.

---

### `get_trim_points`

```matlab
function batch_trim_points = get_trim_points(obj, target_time)
    ...
end
```

- **Description**: Returns trim points closest in time to the target.

---

### `export_trim_points_to_csv`

```matlab
function export_trim_points_to_csv(obj, output_file_path)
    ...
end
```

- **Description**: Writes generated trim points to CSV format.

---

### `export_to_ref_h`

```matlab
function export_to_ref_h(obj, output_file_path)
    ...
end
```

- **Description**: Writes trimmed points to a C-style header file with [time, airspeed, K] format.

---

### `generate_trim_points_from_u_var`

```matlab
function u_vars = generate_trim_points_from_u_var(obj, ...)
    ...
end
```

- **Description**: Internally used to compute velocity spreads with bounded force variation.

---

### `get_info`

```matlab
function [u, thrust] = get_info(obj, time)
    ...
end
```

- **Description**: Interpolates flight data to retrieve velocity and thrust at `time`.

---

### `find_event_time`

```matlab
function event_time = find_event_time(obj, event_name)
    ...
end
```

- **Description**: Searches the CSV for a named event and returns its timestamp.

---

### `look_for_data_types`

```matlab
function positions = look_for_data_types(obj, required_vars)
    ...
end
```

- **Description**: Locates CSV columns matching specified variables.

---

### `trim_rocket_model`

```matlab
function trim_rocket_model(obj, rocket, env, flight, aero)
    ...
end
```

- **Description**: Optimizes state vectors to yield near-steady conditions.

---

### `generate_K`

```matlab
function generate_K(obj, straight_up_Q, straight_up_R, maneuver_Q, maneuver_R, compute_A, compute_B)
    ...
end
```

- **Description**: Computes LQR gain matrices using linearized dynamics at each trimmed point.

---

### `draw_u_trim_points`

```matlab
function draw_u_trim_points(obj, start_time, end_time)
    ...
end
```

- **Description**: Plots real vs. trimmed velocity values over a time window.

---

### `LQR_configuration_selector`

```matlab
function [K, x0, u0] = LQR_configuration_selector(obj, x, t)
    ...
end
```

- **Description**: Given a current state/time, selects the nearest trimmed point and returns its (K, x0, u0).

---

### `compute_controls`

```matlab
function del_u = compute_controls(obj, K, del_x)
    ...
end
```

- **Description**: Simple LQR control law: `del_u = -K * del_x`.

---

### `set_reference_states`

```matlab
function obj = set_reference_states(obj, time_arr, variations_cell_arr)
    ...
end
```

- **Description**: Builds the reference_states struct array for use in feedback control.

---

### `get_reference_state`

```matlab
function reference_state = get_reference_state(obj, time)
    ...
end
```

- **Description**: Retrieves the closest-in-time reference state.

---

### `get_vertical_state` (Static)

```matlab
function full_state = get_vertical_state()
    ...
end
```

- **Description**: Returns a default 13-element vertical flight state.

---

### `cost_rocket_model` (Static)

```matlab
function f0 = cost_rocket_model(Z, trim_point, t, rocket, environment, flight, aero, H)
    ...
end
```

- **Description**: The cost function for state trimming optimization (used by `fminsearch`).

---

## Typical Workflow

```matlab
tp = TrimPoints("my_flight_log.csv");
tp.generate_trim_batches([5, 10, 15]);
tp.add_u_variations(...);
tp.generate_trim_points(rocket, env, flight, aero);
tp.generate_K(...);
tp.export_trim_points_to_csv("output.csv");
```

---

## Conclusion

The `TrimPoints` class is a robust workflow manager for extracting flight data, building rocket state sets, and performing trimming optimizations. It links together data parsing, structured variation, and numerical optimization to prepare data for control design.

---

