# `TrimBatch` Class Documentation

The `TrimBatch` class manages a group of “variations” around a single base state at a specific time. It forms all possible combinations of these variations into distinct trim points for analysis or simulation.

---

## Class Overview

```matlab
classdef TrimBatch < handle
    ...
end
```

- **Inheritance**: Inherits from `handle`
- **Purpose**:
  - Stores a baseline rocket state
  - Collects user-defined variations
  - Forms all combinations into trim points

---

## Properties

- **`state_names` (constant cell array)**  
  All possible state variable names

- **`state_index_con` (constant array)**  
  Indices for `state_names`

- **`variations` (struct array)**  
  Each includes:
  - `name`: ID for the variation
  - `varables`: which state vars are affected
  - `variation_data`: matrix of test values

- **`base_state` (row vector)**  
  Nominal 13-element state vector

- **`time` (double)**  
  Time this batch is anchored to

- **`batch_id` (int)**  
  ID for this batch

- **`num_trim_points` (int)**  
  Number of trim points in this batch

- **`state_dict` (`containers.Map`)**  
  Maps names to indices

- **`trim_points` (struct array)**  
  Final generated trim point structs

---

## Methods

### Constructor

```matlab
function obj = TrimBatch(base_state, time, id)
    ...
end
```

- **Inputs**: 
  - `base_state`: 13×1 state vector
  - `time`: time anchor
  - `id`: batch ID
- **Description**: Initializes the object and sets base state, time, and ID.

---

### `add_variation`

```matlab
function add_variation(obj, name, varables, variation_data)
    ...
end
```

- **Description**: Adds a variation specification to the batch.

---

### `count_trim_points`

```matlab
function count_trim_points(obj)
    ...
end
```

- **Description**: Computes the total number of trim points (Cartesian product size).

---

### `generate_trim_batch`

```matlab
function generate_trim_batch(obj)
    ...
end
```

- **Description**: Forms all trim points by applying all combinations of variations.

---

### `generate_combinations` (Private)

```matlab
function combinations = generate_combinations(obj)
    ...
end
```

- **Description**: Computes index matrix for combination generation.

---

## Typical Usage

```matlab
baseSt = [0 0 0 0 0 0 1 0 0 0 0 0 0];
tb = TrimBatch(baseSt, 5, 1);
tb.add_variation("speed", ["u"], [200; 210; 220]);
tb.generate_trim_batch();
disp(tb.trim_points);
```

---

## Conclusion

The `TrimBatch` class helps manage controlled variations around a nominal rocket state, producing complete trim point sets for trimming, control design, or trajectory initialization.
