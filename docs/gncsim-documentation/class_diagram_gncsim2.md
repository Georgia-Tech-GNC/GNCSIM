# GNCSIM Class Diagram

This document provides a comprehensive overview of the class architecture in GNCSIM, showing how all classes interact and work together to create a complete rocket simulation system.

## Overview

GNCSIM is organized into several packages that handle different aspects of rocket simulation:

- **rocket_pkg**: Core rocket dynamics and aerodynamics
- **controller_pkg**: Control system implementations
- **navigation_pkg**: Navigation and sensor modeling
- **analysis_pkg**: Analysis and Monte Carlo capabilities
- **JVR_files**: Jet vane actuator implementations

## Class Diagram

```mermaid
classDiagram
    %% Main Flight Simulation Classes
    class Flight {
        -rocket_dyn: RocketDynamics
        -controller: Controller
        -environment: Environment
        -nav: Navigation
        -flight_params: struct
        -total_time: double
        -current_time: double
        -dt: double
        +Flight(rocket_dyn, controller, environment, nav, flight_params)
        +run_sim() [t_arr, x_arr]
        +rocket_model(state) x_dot
        +flat_earth_EOM(state, F_b, M_b) state_dot
        +plotFlight(t_arr, x_arr)
    }

    %% Rocket Dynamics
    class RocketDynamics {
        -body: RocketBody
        -rocket_fins: RocketFins
        -motor_params: struct
        -controller: Controller
        -navigation: Navigation
        -mass_wet: double
        -mass_dry: double
        -moi_wet: double[3,3]
        -moi_dry: double[3,3]
        -CG_wet: double
        -CG_dry: double
        +RocketDynamics(body, rocket_fins, rocket_params, motor_params)
        +interp_mass_CG_MOI(t) [m, CG, moi]
        +get_forces_and_moments(state, disturbances, time, breakPoints) [F_b, M_b]
        +compute_buildup()
        +draw()
    }

    %% Aerodynamics Base Class
    class DatcomManager {
        #fileID: int
        #flight_con_params: struct
        +DatcomManager(flight_con_params)
        +open_datcom(datcom_filename)
        +add_flight_condition()
        +close_datcom()
        +run_datcom(datcom_filename)$
    }

    %% Rocket Components
    class RocketBody {
        -body_params: struct
        -body_outline: double[2,n]
        -aero: struct
        +RocketBody(body_params, flight_con_params)
        +get_forces_and_moments(state, disturbances) [F_b, M_b]
        +get_nose_radii(x_nose) r_nose
        +compute_buildup()
        +draw_body()
    }

    class RocketFins {
        -fin_params: struct
        -num_fins: int
        -fins: Fin[]
        -fin_vertices: double[4,3]
        +RocketFins(fin_params, flight_con_params)
        +get_forces_and_moments(state, disturbances) [F_b, M_b]
        +compute_buildup()
        +calculate_ref_geometry(root_chord, tip_chord, semi_span, sweep_rad)
        +draw_fin()
    }

    class Fin {
        -fin_params: struct
        -name: string
        -aero: struct
        +Fin(name, fin_params)
        +get_forces_and_moments(state, disturbances) [F_b, M_b]
    }

    %% Control System
    class Controller {
        <<abstract>>
        #actuator: ActuatorModel
        +get_forces_and_moments(state, disturbances, motor_params, CG, breakPoints, time)*
    }

    class LQR {
        -Q: double[]
        -R: double[]
        -gains: struct
        -actuator: ActuatorModel
        +LQR(Q, R, actuator)
        +compute_gain(A, B) K
        +selectGain(x, t, trimmedPoints, referenceStates) [K, x0, u0]
        +compute_controls(K, del_x) del_u
        +get_forces_and_moments(state, rocket, CG, breakPoints, time) [F_b, M_b]
    }

    class ActuatorModel {
        <<abstract>>
        #location: double[3]
        #q_ba: double[4]
        +get_forces_and_moments(controlInput, rocket, CG)*
    }

    class JetVanesActuatorModel {
        -location: double[3]
        -q_ba: double[4]
        -MT: double
        -biasAng: double[]
        -jetAngs: double[]
        -servoAngs: double[]
        -sideForces: double[]
        -r24: double
        +get_forces_and_moments(controlInput, rocket, CG) [F_b, M_b]
    }

    %% Navigation and Environment
    class Navigation {
        -dt: double
        -x: double[]
        -P: double[]
        -f: function
        -h: function
        -dfdx: function
        -dhdx: function
        -Q: double[]
        -R: double[]
        -K: double[]
        -q: double[]
        -refLLA: double[3]
        -sampleRate: double
        -gpsVerticalAccuracy: double
        -gpsHorizontalAccuracy: double
        +Navigation(state)
        +ekf(inputArg)
        +predictState()
        +kalmanGain() K
        +updateState()
        +updateCovariance() P_posterior
        +predictCovariance() P_prior
        +GPS() gpsRawData
        +Gyro() gyroRawData
        +Accelerometer() accRawData
        +LLA2ECEF(lat, lon, alt) [xECEF, yECEF, zECEF]
        +GPS2NED(lat, lon, alt) [xNED, yNED, zNED]
        +x2GPS() [lat, lon, alt]
    }

    class Environment {
        -vm_mean: double
        -vm_3sigma: double
        -alts: double[]
        -wind_vec: double[]
        -step_size: double
        -max_altitude: double
        +Environment()
        +get_windspeed(alt) windspeed
        +get_vm_mean() vm_mean
        +get_vm_3sigma() vm_3sigma
        +set_vm_mean(vm_mean)
        +set_vm_3sigma(vm_3sigma)
        +get_altitudes() alts
        +set_max_altitude(altitude)
    }

    %% Analysis Classes
    class Analysis {
        +Analysis()
    }

    class MonteCarlo {
        +MonteCarlo()
    }

    %% Utility Class
    class Utils {
        +quatRotate(q, va)$ vb
        +euler_to_quat(a)$ q
        +dir_cosine(q)$ dcm
        +fin_dcm(delta_deg, theta_deg)$ dcm
        +get_forces_and_moments(state, v_c, aero)$ [F_c, M_c]
        +aero_interp(aero, mach, AoA_deg, alt)$ [ca, cm, cn]
    }

    %% Inheritance Relationships
    RocketBody --|> DatcomManager : extends
    RocketFins --|> DatcomManager : extends
    LQR --|> Controller : implements
    JetVanesActuatorModel --|> ActuatorModel : implements

    %% Composition Relationships
    Flight *-- RocketDynamics : contains
    Flight *-- Controller : contains
    Flight *-- Environment : contains
    Flight *-- Navigation : contains

    RocketDynamics *-- RocketBody : contains
    RocketDynamics *-- RocketFins : contains

    RocketFins *-- Fin : contains multiple
    Controller *-- ActuatorModel : contains
    LQR *-- ActuatorModel : contains

    %% Usage/Dependency Relationships
    Flight ..> Utils : uses
    RocketBody ..> Utils : uses
    RocketFins ..> Utils : uses
    Fin ..> Utils : uses
    Navigation ..> Utils : uses

    %% Package Organization
    namespace rocket_pkg {
        class RocketDynamics
        class RocketBody
        class RocketFins
        class Fin
        class DatcomManager
    }

    namespace controller_pkg {
        class Controller
        class LQR
        class ActuatorModel
    }

    namespace navigation_pkg {
        class Navigation
    }

    namespace analysis_pkg {
        class Analysis
        class MonteCarlo
    }

    namespace JVR_files {
        class JetVanesActuatorModel
    }
```

## Architecture Description

### Main Simulation Flow

The **Flight** class serves as the main orchestrator for the entire simulation. It coordinates the interaction between:

1. **RocketDynamics** - Manages the physical properties and aerodynamic forces of the rocket
2. **Controller** - Provides control system logic and actuator commands
3. **Environment** - Models atmospheric conditions and wind
4. **Navigation** - Handles state estimation and sensor modeling

### Key Design Patterns

#### 1. Template Method Pattern
The **DatcomManager** class provides a common framework for generating DATCOM input files, running the DATCOM executable, and importing aerodynamic data. Both **RocketBody** and **RocketFins** inherit from this class to leverage shared DATCOM functionality.

#### 2. Strategy Pattern
Abstract classes **Controller** and **ActuatorModel** allow for different implementations:
- **LQR** implements the Controller interface for Linear Quadratic Regulator control
- **JetVanesActuatorModel** implements the ActuatorModel interface for jet vane actuation

#### 3. Composition over Inheritance
The **Flight** class uses composition rather than inheritance to integrate major subsystems, providing flexibility in swapping implementations.

#### 4. Static Utility Pattern
The **Utils** class provides common mathematical operations (quaternion rotations, coordinate transformations, aerodynamic interpolations) used throughout the system.

### Data Flow

1. **Simulation Loop**: `Flight.run_sim()` uses RK4 integration to propagate the rocket state
2. **Forces Calculation**: `RocketDynamics.get_forces_and_moments()` aggregates aerodynamic and control forces
3. **Aerodynamics**: `RocketBody` and `RocketFins` calculate aerodynamic forces using DATCOM data
4. **Control**: `Controller` implementations generate control commands based on state feedback
5. **Actuation**: `ActuatorModel` implementations convert control commands to forces and moments

### Package Organization

- **rocket_pkg**: Core rocket physics and aerodynamics
- **controller_pkg**: Control system abstractions and implementations
- **navigation_pkg**: Navigation, sensors, and state estimation
- **analysis_pkg**: Analysis tools and Monte Carlo capabilities
- **JVR_files**: Specific actuator implementations (Jet Vane Rocket)

This modular architecture enables easy extension with new control algorithms, actuator models, and analysis capabilities while maintaining clear separation of concerns.