# State Estimation for Rocket Control

## Overview

State estimation is the critical process of determining "where your rocket is and what it's doing" using imperfect sensor data. While LQR control assumes you know all system states perfectly, reality is messier - sensors are noisy, biased, and sometimes measure things indirectly. State estimation bridges this gap by mathematically combining sensor data with system models to produce the best possible estimate of the rocket's current state.

## Why State Estimation is Critical

**The Control Problem**: LQR needs to know position, velocity, orientation, and angular rates to compute optimal control commands. But sensors give us:

- **Noisy measurements**: All sensors have random noise
- **Biased readings**: Sensors have systematic errors and drift over time  
- **Indirect measurements**: Some states (like heading) can't be directly measured in flight
- **Multiple sensors**: Different sensors measure overlapping information with different accuracies

**The Solution**: Use mathematical algorithms to optimally combine all available information - sensor data, system dynamics, and prior knowledge - to estimate the true system state.

**Industry Reality**: Every advanced control system (aircraft, spacecraft, autonomous vehicles) relies heavily on state estimation. It's often more challenging than the control design itself.

---

## 🎯 Core State Estimation Concepts

### The Estimation Problem

**Goal**: Estimate the true state \(x(t)\) given:

- **System model**: \(\dot{x} = f(x, u)\) (how the system evolves)
- **Measurement model**: \(y = h(x) + v\) (what sensors tell us)
- **Process noise**: \(w\) (model uncertainties)
- **Measurement noise**: \(v\) (sensor noise)

### Key Challenges for Rockets

#### Challenge 1: Sensor Characteristics

- **IMU (Inertial Measurement Unit)**: Measures acceleration and angular rates
  - **Issues**: Integration drift, bias, temperature effects
- **GPS**: Provides position and velocity
  - **Issues**: Latency, dropouts, accuracy limits
- **Magnetometer**: Measures magnetic field for heading
  - **Issues**: Local magnetic disturbances, inclination effects
- **Barometer**: Measures altitude
  - **Issues**: Weather variations, dynamic pressure effects

#### Challenge 2: Sensor Fusion

- Multiple sensors measuring related quantities
- Different sampling rates and delays
- Conflicting information requiring resolution
- Optimal weighting based on relative accuracy

#### Challenge 3: Nonlinear Dynamics

- Rocket motion involves rotations (nonlinear)
- Sensor measurements often nonlinear functions of states
- Standard linear techniques may not suffice

### The Kalman Filter Solution

The Kalman filter is the optimal solution for linear systems with Gaussian noise:

**Prediction Step** (based on system model):

$$
\hat{x}^- = A\hat{x} + Bu \quad \text{(Predicted state)}
$$

$$
P^- = APA^T + Q \quad \text{(Predicted uncertainty)}
$$

**Update Step** (incorporating measurements):

$$
K = P^-H^T(HP^-H^T + R)^{-1} \quad \text{(Kalman gain)}
$$

$$
\hat{x} = \hat{x}^- + K(y - H\hat{x}^-) \quad \text{(Updated state)}
$$

$$
P = (I - KH)P^- \quad \text{(Updated uncertainty)}
$$

Where:

- **\(\hat{x}\)**: State estimate
- **\(P\)**: Error covariance (uncertainty)
- **\(Q\)**: Process noise covariance
- **\(R\)**: Measurement noise covariance
- **\(K\)**: Kalman gain (optimal weighting)

---

## 📚 Primary Learning Resource

### Complete Kalman Filter Tutorial

**[Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)**

This is the definitive resource for learning Kalman filtering. **Work through the entire tutorial** - it's comprehensive, practical, and builds intuition while teaching theory.

#### Recommended Chapter Sequence

**Chapters 1-3: Foundation**

- **Chapter 1**: The g-h Filter (intuitive introduction)
- **Chapter 2**: Discrete Bayes Filter (probability fundamentals)
- **Chapter 3**: Probabilities, Gaussians, and Bayes' Theorem (mathematical foundation)

**Chapters 4-6: Core Kalman Theory**

- **Chapter 4**: One-Dimensional Kalman Filter
- **Chapter 5**: Multivariate Kalman Filter  
- **Chapter 6**: Multivariate Kalman Filter Implementation

**Chapters 7-9: Practical Applications**

- **Chapter 7**: Kalman Filter Math
- **Chapter 8**: Designing Kalman Filters
- **Chapter 9**: Nonlinear Filtering

**Chapters 10-12: Advanced Topics**

- **Chapter 10**: Unscented Kalman Filter
- **Chapter 11**: Extended Kalman Filter  
- **Chapter 12**: Particle Filters

**Chapters 13-15: Applications**

- **Chapter 13**: Smoothing
- **Chapter 14**: Adaptive Filtering
- **Chapter 15**: H-infinity Filters

### Study Strategy

- **Don't just read** - run every code example
- **Take detailed notes** - especially on intuitive explanations
- **Modify examples** - try different parameters and scenarios
- **Complete exercises** - they're essential for understanding

---

## 🛠️ Sensor Calibration and Bias Estimation

### Understanding Sensor Errors

#### Turn-On Bias
**Problem**: Sensors read wrong values immediately after power-on
**Example**: Gyroscope reads 0.1 rad/s when rocket is stationary
**Solution**: Calibration routine during pre-flight initialization

#### Drift Over Time
**Problem**: Sensor bias changes during operation
**Example**: Gyroscope bias increases by 0.01 rad/s every minute
**Solution**: Real-time bias estimation using Extended Kalman Filter

#### Temperature Effects
**Problem**: Sensor characteristics change with temperature
**Example**: Accelerometer scale factor varies with temperature
**Solution**: Temperature compensation models

### Our Jet Vanes Implementation

**Extended Kalman Filter for Bias Estimation**:

- **Augmented State**: Include sensor biases as states to be estimated
- **Model**: Biases evolve as random walks or first-order processes
- **Update**: Use redundant information to estimate biases in real-time

**Example Augmented State Vector**:

$$
x = [\theta, \phi, \psi, p, q, r, x, y, z, u, v, w, b_{gx}, b_{gy}, b_{gz}, b_{ax}, b_{ay}, b_{az}]^T
$$

Where \(b_{gi}\) and \(b_{ai}\) are gyroscope and accelerometer biases.

### Calibration Resources
**Research Topics** (no specific resources provided in original document):

- IMU calibration procedures
- Allan variance analysis for sensor characterization
- Six-position calibration for accelerometers
- Rate table calibration for gyroscopes
- Temperature compensation techniques

---

## ✈️ Flight Mechanics for State Estimation

### Why Flight Mechanics Matters

To estimate rocket states, you need to understand:

- **How rockets move**: Equations of motion in 6 degrees of freedom
- **Coordinate systems**: Body, navigation, and inertial reference frames
- **Attitude representation**: Euler angles, quaternions, rotation matrices
- **Sensor placement**: How sensor measurements relate to vehicle states

### Required Learning Resource

**[Christopher Lum - Flight Mechanics Playlist](https://www.youtube.com/playlist?list=PLxdnSsBqCrrEx3A6W94sQGClk6Q4YCg-h)**

#### Critical Videos for State Estimation

**Mathematical Fundamentals**:

- **Videos 1-10**: Coordinate systems, reference frames, transformations
- **Focus**: Understanding how to relate sensor measurements to vehicle states
- **Application**: Converting IMU readings from body frame to navigation frame

**Euler Angles** (Essential):

- **Specific videos on Euler angle sequences**
- **Focus**: How attitude is parameterized and measured
- **Application**: Converting between different attitude representations

**Flat Earth Equations of Motion**:

- **Videos covering 6-DOF equations**
- **Focus**: How vehicle states evolve with control inputs and disturbances
- **Application**: Prediction step of Kalman filter

**Videos 18-21** (Specifically mentioned as important):

- **Advanced kinematics and dynamics concepts**
- **Focus**: Practical aspects of flight mechanics for control
- **Application**: Building accurate system models for estimation

### Key Concepts to Master

#### Reference Frames

- **Inertial Frame**: Fixed to Earth, non-accelerating
- **Navigation Frame**: Local level frame (North-East-Down)
- **Body Frame**: Fixed to rocket, moves with vehicle

#### Attitude Representations

- **Euler Angles**: Intuitive but have singularities
- **Quaternions**: No singularities, used in implementation
- **Direction Cosine Matrix**: Direct transformation, computationally expensive

#### Kinematic Equations

- **How attitude rates relate to Euler angle rates**
- **Quaternion propagation equations**
- **Velocity transformations between reference frames**

---

## 🎯 Optimization and Linearization

### The Linearization Challenge

**Problem**: Kalman filters work with linear systems, but rockets are highly nonlinear
**Solution**: Linearize the nonlinear system around operating points (trim points)

### Required Learning Resource

**[Christopher Lum - Optimization Playlist](https://www.youtube.com/playlist?list=PLxdnSsBqCrrHo2EYb_sMctU959D-iPybT)**

#### Essential Videos: 9-12

**Video 9**: Introduction to Trim Analysis

- **Focus**: What are trim points and why do we need them?
- **Key concept**: Equilibrium conditions for rocket flight
- **Application**: Finding hover conditions, steady climb, etc.

**Video 10**: Setting Up Trim Problems  

- **Focus**: Mathematical formulation of trim constraints
- **Key concept**: Equality and inequality constraints
- **Application**: Specifying desired flight conditions

**Video 11**: Solving Trim Problems

- **Focus**: Numerical methods for constraint optimization
- **Key concept**: Finding control inputs and states that satisfy trim conditions
- **Application**: Computing equilibrium points for linearization

**Video 12**: Linearization at Trim Points

- **Focus**: Computing Jacobian matrices for linear approximation
- **Key concept**: A and B matrices from partial derivatives
- **Application**: Creating linear models for Kalman filter design

### Trim Point Process for Rockets

#### Step 1: Define Trim Conditions
**Hover Trim Example**:

- Vertical position hold: ẋ = 0, ẏ = 0, ż = 0
- Level attitude: θ = 0, φ = 0
- No rotation: p = 0, q = 0, r = 0
- Thrust balances weight: T = mg

#### Step 2: Solve for Trim State and Controls
**Optimization Problem**:

$$
\text{Find: } x_{\text{trim}}, u_{\text{trim}}
$$

$$
\text{Such that: } f(x_{\text{trim}}, u_{\text{trim}}) = 0 \quad \text{(dynamics satisfied)}
$$

$$
g(x_{\text{trim}}, u_{\text{trim}}) = 0 \quad \text{(trim constraints satisfied)}
$$

#### Step 3: Linearize Around Trim Point
**Compute Jacobians**:

$$
A = \frac{\partial f}{\partial x} \bigg|_{(x_{\text{trim}}, u_{\text{trim}})}
$$

$$
B = \frac{\partial f}{\partial u} \bigg|_{(x_{\text{trim}}, u_{\text{trim}})}
$$

**Result**: Linear system \(\dot{x} = Ax + Bu\) valid near trim point

#### Step 4: Design Kalman Filter

- Use linear A and B matrices
- Design appropriate Q and R matrices
- Implement Extended Kalman Filter for nonlinear measurements

---

## 🚀 Rocket-Specific State Estimation

### Typical State Vector

$$
x = \begin{bmatrix}
\theta, \phi, \psi \\ 
p, q, r \\
x, y, z \\  
u, v, w \\
b_{gx}, b_{gy}, b_{gz} \\
b_{ax}, b_{ay}, b_{az}
\end{bmatrix}
$$

Where:

- \(\theta, \phi, \psi\): Euler angles (pitch, roll, yaw)
- \(p, q, r\): Angular rates (body frame)  
- \(x, y, z\): Position (navigation frame)
- \(u, v, w\): Velocity (body frame)
- \(b_{gx}, b_{gy}, b_{gz}\): Gyroscope biases
- \(b_{ax}, b_{ay}, b_{az}\): Accelerometer biases

### Sensor Models

#### IMU Measurements
**Accelerometers**:

$$
a_{\text{meas}} = R_{bn} \cdot (a_{\text{nav}} + g_{\text{nav}}) + b_a + v_a
$$

Where:

- **\(a_{\text{meas}}\)**: Measured acceleration
- **\(R_{bn}\)**: Body-to-navigation rotation matrix
- **\(g_{\text{nav}}\)**: Gravity vector
- **\(b_a\)**: Accelerometer bias
- **\(v_a\)**: Measurement noise

**Gyroscopes**:

$$
\omega_{\text{meas}} = \omega_{\text{body}} + b_g + v_g
$$

#### GPS Measurements

$$
\text{pos}_{\text{meas}} = \begin{bmatrix} x \\ y \\ z \end{bmatrix} + v_{\text{gps}}
$$

$$
\text{vel}_{\text{meas}} = \begin{bmatrix} u \\ v \\ w \end{bmatrix} + v_{\text{gps\_vel}}
$$

### Extended Kalman Filter Implementation

**Prediction Step** (nonlinear):

$$
\hat{x}^- = f(\hat{x}, u, dt) \quad \text{// Integrate nonlinear dynamics}
$$

$$
P^- = FPF^T + Q \quad \text{// F = Jacobian of f}
$$

**Update Step** (linear in measurements):

$$
K = P^-H^T(HP^-H^T + R)^{-1}
$$

$$
\hat{x} = \hat{x}^- + K(y - h(\hat{x}^-))
$$

$$
P = (I - KH)P^-
$$

Where \(h(x)\) is the nonlinear measurement model.

---

## 🎯 Learning Objectives

### Theoretical Mastery

- Understand Bayesian approach to state estimation
- Explain Kalman filter prediction and update steps
- Describe Extended Kalman Filter for nonlinear systems
- Understand sensor fusion and optimal weighting
- Explain bias estimation and calibration concepts

### Mathematical Competency

- Derive Kalman filter equations for simple systems
- Set up measurement models for common sensors
- Compute Jacobians for linearization
- Design Q and R matrices based on physical understanding
- Analyze filter performance and tuning

### Practical Implementation

- Implement Kalman filter for rocket state estimation
- Handle sensor calibration and bias estimation
- Apply flight mechanics to sensor modeling
- Use trim point analysis for linearization
- Debug and tune filter performance

---

## 🛠️ Hands-On Activities

### Activity 1: 1D Kalman Filter
**System**: Simple position tracking with noisy GPS
**Objective**: Build intuition for prediction/update cycle
**Implementation**: Follow tutorial Chapter 4 examples
**Extensions**: Add velocity state, try different Q/R values

### Activity 2: IMU Integration
**System**: Attitude estimation from gyroscope and accelerometer
**Objective**: Understand complementary filtering and sensor fusion
**Challenges**: Handle accelerometer bias and external accelerations
**Extensions**: Add magnetometer for absolute heading

### Activity 3: Rocket State Estimation
**System**: Full 6-DOF rocket with IMU and GPS
**Objective**: Implement Extended Kalman Filter
**Components**:

- Nonlinear dynamics model
- IMU and GPS measurement models
- Bias estimation for sensors
- Real-time implementation considerations

---

## 🔗 Integration with Control Systems

### LQG (Linear Quadratic Gaussian) Control
**Concept**: Combine LQR control with Kalman filter state estimation
**Structure**:

$$
\text{Sensors} \rightarrow \text{Kalman Filter} \rightarrow \text{State Estimate} \rightarrow \text{LQR Controller} \rightarrow \text{Actuators}
$$
**Separation Principle**: Can design estimator and controller independently

### Practical Considerations

- **Computation**: Real-time implementation constraints
- **Robustness**: Handle sensor failures and model uncertainties  
- **Performance**: Balance estimation accuracy with computational cost
- **Safety**: Implement monitoring and failure detection

---

## 🚨 Common Challenges and Solutions

### Challenge 1: Filter Divergence
**Problem**: Estimated state drifts away from true state
**Causes**:

- Poor model accuracy
- Incorrect Q/R tuning
- Numerical issues

**Solutions**:

- Improve system model fidelity
- Increase process noise (Q) if model is uncertain
- Use square root filtering for numerical stability
- Implement innovation monitoring

### Challenge 2: Sensor Bias Estimation
**Problem**: Difficult to distinguish between true motion and sensor bias
**Causes**:

- Insufficient excitation
- Observability issues
- Conflicting sensor information

**Solutions**:

- Design flight maneuvers for bias observability
- Use multiple sensor types for redundancy
- Implement bias estimation with appropriate time constants
- Apply Allan variance analysis for characterization

### Challenge 3: Attitude Singularities
**Problem**: Euler angles have singularities (gimbal lock)
**Causes**:

- Pitch angle approaches ±90°
- Mathematical representation breakdown

**Solutions**:

- Use quaternions for attitude representation
- Switch between Euler angle sets dynamically
- Implement modified Rodrigues parameters
- Design flight profiles to avoid singularities

### Challenge 4: Computational Burden
**Problem**: Real-time constraints on flight computer
**Causes**:

- Large state vectors
- Nonlinear measurement updates
- Matrix inversions

**Solutions**:

- Use reduced-order filters when possible
- Implement efficient matrix operations
- Consider distributed filtering architectures
- Apply measurement decimation strategies

---

## 🎯 Advanced State Estimation Topics

### Unscented Kalman Filter (UKF)
**Advantage**: Better handling of nonlinearities than EKF
**Concept**: Use sigma points to capture nonlinear transformations
**Application**: Highly nonlinear measurement models
**Trade-off**: More computation vs. better accuracy

### Particle Filters
**Advantage**: Can handle arbitrary nonlinearities and non-Gaussian noise
**Concept**: Represent probability distribution with particles
**Application**: Highly nonlinear/non-Gaussian problems
**Trade-off**: Much more computation required

### Adaptive Filtering
**Problem**: Q and R matrices may not be constant
**Solution**: Adapt noise parameters based on innovation sequence
**Methods**: Multiple model adaptive estimation, maximum likelihood
**Application**: Changing flight conditions, sensor degradation

### Fault Detection and Isolation
**Objective**: Detect and isolate sensor failures
**Methods**:

- Innovation-based detection
- Multiple model approaches
- Parity space methods
**Critical for**: Flight safety and mission success

---

## 🔬 Research and Development Opportunities

### Current GNC Challenges

1. **Multi-sensor Integration**: Optimal fusion of IMU, GPS, magnetometer, barometer
2. **Real-time Performance**: Meeting computational constraints on flight hardware
3. **Robustness**: Handling sensor failures and environmental variations
4. **Calibration**: Automated sensor calibration procedures

### Future Directions

1. **Machine Learning Integration**: Using neural networks for improved models
2. **Distributed Estimation**: Multiple vehicles sharing information
3. **Vision-based Navigation**: Integrating cameras for position estimation
4. **Advanced Sensors**: LiDAR, radar, optical flow sensors

---

## 🎯 Self-Assessment Checklist

### Conceptual Understanding

- Can explain Bayesian approach to estimation in simple terms
- Understand why Kalman filter is "optimal" and limitations of optimality
- Can describe difference between prediction and update steps physically
- Know when to use EKF vs. UKF vs. particle filters
- Understand sensor fusion and information weighting

### Mathematical Competency

- Can derive Kalman filter equations for simple systems
- Know how to linearize nonlinear dynamics around trim points
- Can set up measurement models for IMU and GPS
- Understand covariance propagation and uncertainty quantification
- Can analyze observability and controllability

### Implementation Skills

- Can implement linear Kalman filter from scratch
- Know how to tune Q and R matrices systematically
- Can handle practical issues like initialization and numerical stability
- Understand real-time implementation constraints
- Can debug filter performance problems

### System Integration

- Know how to combine state estimation with LQR control
- Understand sensor calibration procedures and bias estimation
- Can apply flight mechanics to rocket state estimation
- Know how to handle coordinate transformations
- Understand safety and fault tolerance requirements

---

## 🏆 Mastery Indicators

You've mastered state estimation when you can:

1. **Design Complete Systems**: Set up Kalman filter for arbitrary dynamic system with multiple sensors
2. **Handle Real-World Issues**: Address sensor biases, noise, failures, and computational constraints
3. **Optimize Performance**: Systematically tune filter parameters for desired performance
4. **Debug Problems**: Identify and solve filter divergence, poor estimation, or implementation issues
5. **Integrate with Control**: Seamlessly combine estimation with LQR for complete control system

---

## 🔗 Connection to Team Success

### Critical Mission Functions

- **Trajectory Tracking**: Know where rocket is relative to planned path
- **Attitude Control**: Precise orientation for thrust vector control
- **Landing Guidance**: Accurate position and velocity for powered landing
- **Safety Monitoring**: Detect anomalies and trigger abort sequences

### Technical Leadership Opportunities

- **Algorithm Development**: Improve estimation accuracy and robustness
- **Hardware Integration**: Interface with new sensors and flight computers
- **Performance Analysis**: Characterize and improve system performance
- **Mentoring**: Help new team members understand estimation concepts

---

## 💡 Key Takeaways

### Fundamental Principles

1. **Information Fusion**: Optimally combine all available information
2. **Uncertainty Quantification**: Know how confident you are in estimates
3. **Model-Based Approach**: Use physics to improve estimation
4. **Recursive Processing**: Update estimates as new data arrives
5. **Robustness**: Design for real-world imperfections

### Engineering Insights

1. **Sensors are Imperfect**: Always account for noise, bias, and failures
2. **Models are Approximations**: Balance complexity vs. computational cost
3. **Tuning is Critical**: Q and R matrices dramatically affect performance
4. **Integration Matters**: Estimation and control must work together
5. **Testing is Essential**: Simulation and hardware validation required

### Career Relevance
State estimation skills are valuable across industries:

- **Aerospace**: Aircraft, spacecraft, and UAV navigation
- **Automotive**: Self-driving cars and advanced driver assistance
- **Robotics**: Mobile robots, manipulators, and autonomous systems
- **Industrial**: Process monitoring and predictive maintenance

---

## 🚀 Next Steps

### Immediate Actions

1. **Start the Tutorial**: Begin working through Kalman and Bayesian Filters in Python
2. **Join Study Groups**: Connect with other team members learning estimation
3. **Practice Implementation**: Code examples and experiment with parameters
4. **Ask Questions**: Engage with team leads for guidance and clarification

### Integration Preparation

1. **Review LQR Material**: Understand how estimation feeds into control
2. **Study Flight Software**: Learn how estimation fits into overall system
3. **Hardware Familiarity**: Understand sensors and computational platform
4. **Safety Considerations**: Learn fault detection and abort procedures

### Long-term Development

1. **Advanced Methods**: Study nonlinear filtering and adaptive techniques
2. **Research Opportunities**: Contribute to estimation algorithm improvements
3. **Cross-Training**: Learn other GNC disciplines for system perspective
4. **Industry Preparation**: Build portfolio demonstrating estimation expertise

---

## 📚 Additional Resources

### Textbooks (for deeper study)

- **"Optimal State Estimation" by Dan Simon**: Comprehensive mathematical treatment
- **"Stochastic Models, Estimation, and Control" by Maybeck**: Classic reference
- **"Applied Optimal Estimation" by Gelb**: Practical aerospace applications

### Software Tools

- **MATLAB/Simulink**: Industry standard for filter development and simulation
- **Python**: Excellent for learning and rapid prototyping
- **C/C++**: For real-time embedded implementation
- **ROS**: Robot Operating System with built-in estimation packages

### Professional Development

- **IEEE Control Systems Society**: Professional organization and conferences
- **AIAA Guidance, Navigation, and Control Conference**: Premier aerospace venue
- **Online Courses**: Coursera, edX courses on estimation and sensor fusion
- **Industry Workshops**: Boeing, Lockheed, SpaceX often offer technical training

---

## 🎯 Remember the Big Picture

State estimation is the **eyes and ears** of your control system. Without accurate state knowledge, even the most sophisticated LQR controller will fail. Your work in state estimation directly enables:

- **Precision Control**: LQR needs accurate states to compute optimal commands
- **Mission Success**: Navigation and guidance depend on knowing where you are
- **Safety**: Abort decisions require reliable state information
- **Performance**: Better estimation enables more aggressive control

The mathematical complexity can be daunting, but remember: you're solving a fundamental problem that enables autonomous systems to perceive and interact with the world. Every spacecraft, autonomous vehicle, and advanced robot relies on the principles you're learning.

**Master state estimation, and you'll have a superpower that's in demand across the entire aerospace and robotics industry.**