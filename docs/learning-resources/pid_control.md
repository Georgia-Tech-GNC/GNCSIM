# PID Control Fundamentals

## Overview

PID (Proportional-Integral-Derivative) control is a fundamental control strategy that forms the backbone of most industrial control systems. Understanding PID is essential because it teaches you the core concepts of closed-loop control that underlie all control systems.

## Why Learn PID Control?

**Foundation of Control Theory**: PID introduces you to closed-loop control, feedback, and system response concepts that are universal across all control methods.

**Industry Standard**: Over 95% of industrial control systems use some form of PID control. Understanding it makes you valuable in any controls-related career.

**Intuitive Learning**: PID is relatively simple to understand conceptually, making it an excellent stepping stone to more advanced control techniques like LQR.

**GNC Heritage**: We've successfully used PID on our gimbaled rocket launches, so understanding it helps you appreciate our team's control systems evolution.

---

## 🎛️ Core PID Concepts

### The Control Problem
Imagine you want your rocket to point in a specific direction:

- **Setpoint**: Where you want the rocket to point (reference)
- **Process Variable**: Where the rocket is actually pointing (measured output)
- **Error**: The difference between where you want to be and where you are
- **Control Output**: The command sent to actuators (like gimbal motors)

### The Three Components

#### Proportional (P)

- **What it does**: Provides control output proportional to the current error
- **Mathematical form**: \(P = K_p \times \text{error}\)
- **Physical meaning**: "Push harder when you're further from the target"
- **Pros**: Simple, responsive
- **Cons**: Cannot eliminate steady-state error, can cause oscillations

#### Integral (I)

- **What it does**: Provides control output based on the accumulated error over time
- **Mathematical form**: \(I = K_i \times \int \text{error} \, dt\)
- **Physical meaning**: "Remember how long you've been off-target and correct for it"
- **Pros**: Eliminates steady-state error
- **Cons**: Can cause overshoot, slow response

#### Derivative (D)

- **What it does**: Provides control output based on the rate of change of error
- **Mathematical form**: \(D = K_d \times \frac{d(\text{error})}{dt}\)
- **Physical meaning**: "Anticipate where the error is heading and act early"
- **Pros**: Improves stability, reduces overshoot
- **Cons**: Amplifies noise, can cause jerky control

### Complete PID Equation

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

---

## 📚 Learning Resources

### Primary Video Series
**Understanding PID Control Playlist (Videos 1-5)**
[What Is PID Control? | Understanding PID Control, Part 1](https://www.youtube.com/watch?v=wkfEZmsQqiA&list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y)

Work through these videos in sequence.

### Supplementary Resource
**[Introduction to PID Control - Christopher Lum](https://www.youtube.com/watch?v=_VzHpLjKeZ8&list=PLxdnSsBqCrrF9KOQRB9ByfB0EUMwnLO9o&index=20)**

- Provides a more mathematical perspective
- Good for reinforcing concepts from the main series

---

## 🎯 Learning Objectives

By the end of this section, you should be able to:

### Conceptual Understanding

- Explain what each component of PID does in physical terms
- Describe when you would increase/decrease each gain
- Identify the symptoms of poor PID tuning
- Understand the trade-offs between performance and stability

### Mathematical Competency

- Write the PID equation in time domain and s-domain
- Sketch the response of each component to step and ramp inputs
- Explain how PID parameters affect closed-loop poles
- Connect PID behavior to transfer function analysis

### Practical Application

- Propose a tuning strategy for a given system
- Identify when PID is appropriate vs. when more advanced control is needed
- Understand how to implement PID in discrete-time (for embedded systems)
- Recognize and address common implementation issues

---

## 🔧 Hands-On Learning Activities

### Activity 1: PID Simulation
**Objective**: Build intuition for how each parameter affects system response

**Tools needed**: MATLAB/Simulink, Python, or any simulation environment

**MATLAB Code**:
```matlab
clear; close all; clc;

% Define a simple second-order system (rocket attitude dynamics)
% Transfer function: G(s) = 1/(s^2 + 2*zeta*wn*s + wn^2)
wn = 2;      % Natural frequency
zeta = 0.3;  % Damping ratio
num = wn^2;
den = [1, 2*zeta*wn, wn^2];
G = tf(num, den);

% PID Controller gains (start here and experiment!)
Kp = 1;    % Proportional gain
Ki = 0;    % Integral gain  
Kd = 0;    % Derivative gain

% Create PID controller
C = pid(Kp, Ki, Kd);

% Closed-loop system
T = feedback(C*G, 1);

% Step response
figure;
step(T);
grid on;
title(['PID Response: Kp=', num2str(Kp), ', Ki=', num2str(Ki), ', Kd=', num2str(Kd)]);
xlabel('Time (s)');
ylabel('Response');

% Display system info
stepinfo(T)
```

**Experimentation Steps**:

1. **P-only control**: Start with `Kp = 1`, `Ki = 0`, `Kd = 0`
2. **Add Integral**: Try `Kp = 1`, `Ki = 0.5`, `Kd = 0` 
3. **Add Derivative**: Try `Kp = 1`, `Ki = 0.5`, `Kd = 0.1`
4. **Experiment**: Change gains and observe the response

**Key insights to discover**:

- Higher \(K_p\) → faster response but more oscillation
- Higher \(K_i\) → no steady-state error but potential overshoot
- Higher \(K_d\) → smoother response but noise amplification

### Activity 2: Manual Tuning Practice
**Objective**: Develop intuitive tuning skills

**Scenario**: Rocket attitude control with PID

**MATLAB Code for Tuning**:
```matlab
clear; close all; clc;

% System: Rocket attitude dynamics with disturbance
num = 1;
den = [1, 0.5, 1];  % Slightly different system for tuning practice
G = tf(num, den);

% Step 1: Start with P-only control
Kp = 1; Ki = 0; Kd = 0;
C = pid(Kp, Ki, Kd);
T1 = feedback(C*G, 1);

% Step 2: Increase Kp until oscillation
Kp = 3; Ki = 0; Kd = 0;  % Try higher Kp values
C = pid(Kp, Ki, Kd);
T2 = feedback(C*G, 1);

% Step 3: Back off Kp and add Ki
Kp = 1.5; Ki = 1; Kd = 0;  % Reduce Kp, add Ki
C = pid(Kp, Ki, Kd);
T3 = feedback(C*G, 1);

% Step 4: Add Kd to improve response
Kp = 1.5; Ki = 1; Kd = 0.2;  % Add derivative
C = pid(Kp, Ki, Kd);
T4 = feedback(C*G, 1);

% Compare all responses
figure;
step(T1, T2, T3, T4, 0:0.1:15);
legend('P only', 'High Kp', 'PI', 'PID', 'Location', 'best');
grid on;
title('PID Tuning Evolution');
```

**Tuning Steps**:

- **Step 1**: Start with \(K_p = 1\), \(K_i = 0\), \(K_d = 0\)
- **Step 2**: Increase \(K_p\) until you get sustained oscillation
- **Step 3**: Back off \(K_p\) by 50%, add \(K_i\) slowly until steady-state error vanishes
- **Step 4**: Add \(K_d\) to reduce overshoot

**Document**: How the response changes with each adjustment

---

## 🎨 PID in GNC Context

### Our Gimbal Control Experience

- **System**: Rocket attitude control via gimbal deflection
- **Sensors**: IMU providing attitude and angular rate feedback
- **Actuators**: Servo motors controlling gimbal angles
- **Challenges**: Noise in sensors, actuator saturation, wind disturbances

### Why We Moved to LQR for the Jet Vanes Rocket
While PID worked for our gimbal system, we upgraded to LQR for jet vanes because:

1. **Multi-input, Multi-output**: Jet vanes require coordinating multiple control surfaces
2. **Optimal Performance**: LQR mathematically optimizes the control law
3. **State Feedback**: LQR uses full system state, not just error feedback
4. **Robustness**: Better handling of system uncertainties and disturbances

### PID Legacy Knowledge
Understanding PID helps you appreciate:

- Why feedback control is necessary
- How control systems can become unstable
- The importance of sensor noise and actuator limitations
- Trade-offs between performance and robustness

---

## 🚀 Common PID Challenges in Rocket Control

### Challenge 1: Actuator Saturation
**Problem**: Control commands exceed actuator limits
**PID Issue**: Integral windup can cause large overshoots
**Solution**: Anti-windup techniques, output limiting

### Challenge 2: Sensor Noise
**Problem**: Noisy sensor readings, especially affecting derivative term
**PID Issue**: D-term amplifies high-frequency noise
**Solution**: Low-pass filtering, derivative on measurement

### Challenge 3: Changing System Dynamics
**Problem**: Rocket mass changes as fuel burns, aerodynamics change with altitude
**PID Issue**: Fixed gains may not work across all flight conditions
**Solution**: Gain scheduling, adaptive control (leads to advanced techniques like LQR)

### Challenge 4: Multiple Objectives
**Problem**: Want to control multiple things simultaneously (attitude, position, velocity)
**PID Issue**: Single-loop PID can't easily handle multiple objectives
**Solution**: Move to multi-variable control methods (LQR, MPC)

---

## 🔗 Connection to Advanced Control

Understanding PID sets you up for success with more advanced control techniques:

### To State-Space Control

- **Feedback concept**: Directly translates to state feedback
- **Performance trade-offs**: Similar considerations in LQR design
- **Implementation issues**: Many carry over to advanced controllers

### To LQR Control

- **Control objectives**: PID tries to minimize error; LQR optimizes a cost function
- **Systematic design**: LQR provides mathematical framework for what PID does intuitively
- **Multi-variable extension**: LQR naturally handles multiple inputs and outputs

### Industry Perspective

- Most industrial systems still use PID for simplicity and reliability
- Advanced control (like LQR) is used when PID limitations become apparent
- Understanding both makes you versatile in any controls engineering role

---

## 💡 Key Takeaways

1. **PID is intuitive**: Each component has a clear physical interpretation
2. **Tuning is an art and science**: Systematic approaches exist, but experience matters
3. **Limitations drive innovation**: Understanding PID's limits motivates learning advanced control
4. **Foundation for everything**: Concepts from PID appear in all control methods
5. **Still widely used**: Don't dismiss PID as "old" - it's still the industry workhorse

**Next Step**: With PID mastery, you're ready to tackle **State-Space Representation**, where we'll see how modern control systems are mathematically structured for optimal performance.