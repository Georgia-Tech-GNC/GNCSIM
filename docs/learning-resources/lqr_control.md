# Linear Quadratic Regulator (LQR) Control

## Overview

Linear Quadratic Regulator (LQR) control is the advanced control technique we implemented on our jet vanes rocket. Unlike PID control, which uses simple error-based feedback, LQR is an optimal control method that mathematically determines the best possible control law for a given system. It represents the pinnacle of classical control theory and is widely used in aerospace, robotics, and advanced industrial applications.

## Why LQR for Jet Vanes?

**Our Evolution**: We successfully used PID for gimbal control, but jet vanes demand better performance:

- **Multi-variable System**: Jet vanes require coordinated control of multiple surfaces
- **Optimal Performance**: LQR mathematically guarantees the best possible performance for given design criteria
- **Full State Feedback**: Uses all available system information, not just error
- **Systematic Design**: Replaces intuitive PID tuning with rigorous mathematical optimization
- **Robustness**: Better handling of uncertainties and disturbances

**Industry Standard**: LQR is the foundation of modern spacecraft attitude control, aircraft autopilots, and advanced robotic systems.

---

## 🎯 LQR Fundamentals

### The Optimal Control Problem

LQR solves this question: **"What control input minimizes a mathematical cost function while satisfying system dynamics?"**

#### System Model (State-Space)

$$
\dot{x} = Ax + Bu \quad \text{(System dynamics)}
$$

$$
y = Cx + Du \quad \text{(Measurement equation)}
$$

Where:

- **x**: State vector (positions, velocities, angles, angular rates)
- **u**: Control input vector (jet vane deflections)
- **A, B, C, D**: System matrices describing dynamics

#### Cost Function

$$
J = \int_0^\infty (x^T Q x + u^T R u) \, dt
$$

Where:

- **Q**: State weighting matrix (how much you care about each state)
- **R**: Control weighting matrix (how much you care about control effort)
- **J**: Total cost to minimize

#### Optimal Control Law
The LQR solution is always linear state feedback:

$$
u = -Kx
$$

Where \(K\) is the optimal gain matrix computed by solving the Riccati equation.

### Physical Interpretation

**Q Matrix**: "How much do I care about each state?"

- Large Q values → tight control of those states
- Small Q values → relaxed control, allow more deviation

**R Matrix**: "How expensive is control effort?"

- Large R → gentle, conservative control
- Small R → aggressive, high-energy control

**K Matrix**: The optimal compromise between performance and control effort

- Automatically balances Q and R requirements
- Provides mathematically optimal feedback gains

---

## 📚 Learning Resources

### Core LQR Theory

#### Primary Video Sequence
Work through these in order to build understanding:

**1. [What is Pole Placement (Full State Feedback) | State Space, Part 2](https://www.youtube.com/watch?v=FXSpHy8LvmY)**

**2. [A Conceptual Approach to Controllability and Observability | State Space, Part 3](https://www.youtube.com/watch?v=BYvTEfNAi38)**


**3. [What Is Linear Quadratic Regulator (LQR) Optimal Control? | State Space, Part 4](https://www.youtube.com/watch?v=E_RDCFOlJx4)**

#### Detailed Implementation Resources

**4. [Controllability of a Linear System: The Controllability Matrix and the PBH Test](https://www.youtube.com/watch?v=oQDi3Giv-DI&list=PLxdnSsBqCrrF9KOQRB9ByfB0EUMwnLO9o&index=26)**

**5. [Introduction to Full State Feedback Control](https://www.youtube.com/watch?v=1zIIcYfp5QA&list=PLxdnSsBqCrrF9KOQRB9ByfB0EUMwnLO9o&index=27)**


**6. [Introduction to Linear Quadratic Regulator (LQR) Control](https://www.youtube.com/watch?v=wEevt2a4SKI&list=PLxdnSsBqCrrF9KOQRB9ByfB0EUMwnLO9o&index=29)**


---

## 🚀 LQR for Rocket Control

### Jet Vanes System Model

For our rocket, the state vector typically includes:

$$
x = [\theta, \phi, \psi, p, q, r, x, y, z, u, v, w]^T
$$

Where:

- **θ, φ, ψ**: Euler angles (pitch, roll, yaw)
- **p, q, r**: Angular rates
- **x, y, z**: Position
- **u, v, w**: Velocities

Control inputs:

$$
u = [\delta_1, \delta_2, \delta_3, \delta_4]^T
$$

Where \(\delta_i\) are the four jet vane deflection angles.

### Design Process

#### Step 1: System Linearization

- Start with nonlinear rocket dynamics
- Find trim points (equilibrium conditions)
- Linearize around trim points using Jacobians
- Results in linear state-space model: \(\dot{x} = Ax + Bu\)

#### Step 2: LQR Weight Selection
**Q Matrix Design** (State Penalties):

- Attitude errors: High weights for tight pointing control
- Position errors: Medium weights for trajectory following
- Rate errors: Moderate weights to prevent excessive motion

**R Matrix Design** (Control Penalties):

- Balance control authority vs. actuator wear
- Consider power consumption and actuator limits
- Prevent chattering and high-frequency commands

#### Step 3: Gain Computation

- Solve Algebraic Riccati Equation (ARE)
- MATLAB: `K = lqr(A, B, Q, R)`
- Verify closed-loop stability
- Check performance through simulation

#### Step 4: Implementation Considerations

- Handle actuator saturation
- Account for sensor noise and delays
- Implement gain scheduling for different flight phases
- Add integral action if needed for disturbance rejection

---

## 🎯 Learning Objectives

By completing this section, you should master:

### Theoretical Understanding

- Explain the LQR optimal control problem in physical terms
- Understand the role of Q and R matrices in design
- Describe how LQR relates to pole placement
- Explain controllability and its importance
- Understand the Riccati equation conceptually

### Mathematical Competency

- Set up the LQR problem for a given system
- Choose appropriate Q and R matrices for design objectives
- Verify system controllability
- Compute LQR gains using mathematical tools
- Analyze closed-loop performance and stability

### Practical Application

- Design LQR controller for rocket attitude control
- Handle implementation issues (saturation, noise, discretization)
- Tune Q and R for desired performance
- Compare LQR vs. PID performance
- Understand when LQR is appropriate vs. other methods

---

## 🛠️ Hands-On Learning Activities

### Activity 1: Simple LQR Design
**System**: Double integrator (simplified rocket attitude)

$$
\dot{x}_1 = x_2 \quad \text{(angle)}
$$

$$
\dot{x}_2 = u \quad \text{(angular acceleration)}
$$

**Tasks**:

1. Write state-space matrices A and B
2. Choose Q and R matrices
3. Compute LQR gains
4. Simulate step response
5. Compare different Q and R choices

**Learning Goals**:

- Build intuition for Q/R selection
- See how LQR gains depend on weights
- Compare to PID performance

### Activity 2: Rocket Attitude Control
**System**: 6-DOF rocket attitude dynamics

**Tasks**:

1. Use linearized rocket model around hover trim point
2. Design Q matrix emphasizing attitude control
3. Design R matrix balancing control effort
4. Compute and analyze LQR gains
5. Simulate attitude control performance
6. Add disturbances and assess robustness

**Learning Goals**:

- Work with realistic system complexity
- Handle multiple inputs and outputs
- See LQR performance on actual rocket dynamics

### Activity 3: Performance Comparison
**Objective**: Compare LQR vs. PID for same system

**Tasks**:

1. Design PID controller for rocket attitude
2. Design LQR controller for same system
3. Compare step responses
4. Compare disturbance rejection
5. Analyze robustness to model uncertainties
6. Evaluate computational requirements

**Learning Goals**:

- Understand when LQR advantages are worth the complexity
- See performance trade-offs between methods
- Build engineering judgment for controller selection

---

## 🔬 Advanced LQR Concepts

### LQR with Integral Action (LQI)
**Problem**: Pure LQR has no steady-state error rejection for constant disturbances

**Solution**: Augment system with integrator states

$$
x_{\text{aug}} = \begin{bmatrix} x \\ \int e \, dt \end{bmatrix}
$$

**Result**: LQI combines LQR optimality with integral action

### Gain Scheduling
**Problem**: Rocket dynamics change dramatically during flight

**Solution**: Design multiple LQR controllers for different flight conditions

- Hover phase: Different A, B matrices
- Ascent phase: Different linearization point
- Transition smoothly between controllers

### Robust LQR
**Problem**: Model uncertainties can degrade performance

**Extensions**:

- \(H_\infty\) control for worst-case robustness
- μ-synthesis for structured uncertainties
- Adaptive LQR for parameter variations

---

## 🎨 LQR Design Philosophy

### The Art of Weight Selection

**Q Matrix Guidelines**:

- Start with identity, then iterate
- Higher values → tighter control of those states
- Balance different units (angles vs. positions vs. rates)
- Consider coupled effects between states

**R Matrix Guidelines**:

- Start with identity scaled by actuator limits
- Higher values → more conservative control
- Balance multiple actuators appropriately
- Consider actuator bandwidth and power limits

**Iterative Design Process**:

1. Start with reasonable initial weights
2. Simulate closed-loop performance
3. Identify performance shortcomings
4. Adjust weights systematically
5. Re-simulate and evaluate
6. Repeat until satisfied

### Engineering Judgment
LQR is mathematically optimal, but optimality depends on your choices:

- **Model accuracy**: LQR is only as good as your linear model
- **Weight selection**: No automatic way to choose Q and R
- **Implementation constraints**: Real systems have limits LQR doesn't know about

---

## 🚨 Common Pitfalls and Solutions

### Pitfall 1: Poor Weight Selection
**Problem**: Unrealistic Q and R matrices lead to poor performance
**Symptoms**: Oscillatory response, excessive control effort, poor tracking
**Solution**: Use physical insight, start simple, iterate systematically

### Pitfall 2: Uncontrollable System
**Problem**: Trying to control states that can't be controlled
**Symptoms**: LQR computation fails or gives infinite gains
**Solution**: Check controllability matrix, redesign system if needed

### Pitfall 3: Ignoring Implementation Limits
**Problem**: LQR assumes unlimited control authority
**Symptoms**: Actuator saturation, performance degradation
**Solution**: Include actuator models, use anti-windup, consider MPC

### Pitfall 4: Over-Reliance on Linearity
**Problem**: Real rockets are highly nonlinear
**Symptoms**: Performance degrades away from trim point
**Solution**: Use gain scheduling, adaptive control, or nonlinear methods

### Pitfall 5: Ignoring Sensor Limitations
**Problem**: LQR assumes perfect state measurement
**Symptoms**: Noise amplification, poor practical performance
**Solution**: Combine with state estimation (Kalman filter), filter derivatives

---

## 🎯 Self-Assessment Checklist

### Conceptual Mastery

- Can explain why LQR is "optimal" and what that means
- Understand the physical meaning of Q and R matrices  
- Can describe the relationship between LQR and pole placement
- Know when LQR is appropriate vs. other control methods
- Understand controllability and its importance

### Mathematical Competency

- Can set up LQR problem from system dynamics
- Know how to check controllability
- Can interpret Riccati equation solution
- Understand closed-loop eigenvalue placement
- Can analyze stability and performance mathematically

### Practical Skills

- Can design LQR controller for multi-variable system
- Know how to choose Q and R matrices systematically
- Can implement LQR in simulation and hardware
- Understand gain scheduling and robust design
- Can compare LQR vs. classical control performance

---

## 🔗 Integration with GNC Systems

### Connection to State Estimation
**Challenge**: LQR assumes you know all states perfectly
**Reality**: Must estimate states from noisy sensor data
**Solution**: Combine LQR with Kalman filter (LQG - Linear Quadratic Gaussian)

### Connection to Flight Software
**Implementation Requirements**:
- Real-time computation of control law
- Handling of sensor data and actuator commands
- Safety checks and failure modes
- Discrete-time implementation

### Connection to Hardware
**Practical Considerations**:
- Actuator bandwidth and saturation limits
- Sensor noise and sampling rates
- Computational constraints on flight computer
- Power consumption and thermal management

---

## 🏆 Why This Matters

### Academic Impact
- **Resume Value**: LQR knowledge demonstrates advanced controls competency
- **Graduate Studies**: Foundation for modern control theory and robotics
- **Research Applications**: Used in cutting-edge aerospace and robotics research

### Industry Relevance
- **Aerospace**: Spacecraft attitude control, launch vehicle guidance
- **Automotive**: Advanced driver assistance, autonomous vehicles
- **Robotics**: Manipulation, locomotion, aerial robotics
- **Industrial**: Process control, manufacturing automation

### GNC Team Impact
- **Project Success**: LQR enables advanced rocket control capabilities
- **Technical Leadership**: Understanding LQR makes you a technical contributor
- **Problem Solving**: Provides mathematical framework for control design
- **Innovation**: Foundation for even more advanced control techniques

---

## 🎓 Mastery Indicators

You've mastered LQR when you can:

1. **Explain to a teammate**: Describe LQR benefits over PID for jet vanes rocket
2. **Design from scratch**: Set up and solve LQR problem for new system
3. **Debug performance issues**: Diagnose problems and adjust Q/R matrices appropriately
4. **Implement practically**: Handle real-world constraints and implementation challenges
5. **Make design trade-offs**: Balance performance, robustness, and computational requirements

**Next Steps**: With LQR mastery, you're ready to tackle **State Estimation**, where you'll learn how to obtain the state information that LQR requires through Kalman filtering and sensor fusion.

---

## 💡 Final Thoughts

LQR represents a quantum leap from classical control methods like PID. It transforms control design from an art based on intuition to a science based on optimization. For our jet vanes rocket, LQR provides the mathematical rigor needed to achieve precision control in a challenging environment.

The journey from PID to LQR parallels the evolution of engineering itself - from empirical methods to mathematical optimization. Master LQR, and you'll have a powerful tool for any advanced control challenge you encounter in your career.