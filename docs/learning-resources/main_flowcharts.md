# Controls Team Learning Path

## Why This Matters

Controls is a difficult subject that requires a lot of prerequisite knowledge to meaningfully contribute to a project such as the jet vanes rocket we're building. The GNC (Guidance, Navigation, and Control) team works on complex rocket control systems that demand interdisciplinary knowledge spanning mathematics, engineering, and computer science.

Taking the time to learn these fundamentals will benefit you not only as a member of GNC, but also as a student and for your resume. At Georgia Tech, the traditional path to controls knowledge requires taking PHYS 2211, COE 2001, AE 2220/MATH 2552, AE 3530, and finally AE 3531 - meaning it takes at least two years from physics to reach control theory. This bootcamp condenses that knowledge into focused, practical learning paths.


---

## 🎯 Choose Your Learning Path

```mermaid
flowchart TD
    Start([New GNC Member]) --> Assessment{What interests you most?}
    
    Assessment --> Controls[🎮 Controls<br/>Making rockets respond to commands<br/>PID Implementation<br/>LQR Implementation]
    Assessment --> State[📡 State Estimation<br/>Knowing where the rocket is<br/>Kalman Filtering]
    
    Controls --> ControlsFlow[See Controls Flowchart Below]
    State --> StateFlow[See State Estimation Flowchart Below]
    
    style Controls fill:#e3f2fd
    style State fill:#f3e5f5
```

---

## 🎮 Controls Learning Path

**Goal**: Learn from the ground-up until Linear Quadratic Regulator (LQR) control

```mermaid
flowchart TB
    A[📚 Mathematical Foundation<br/>Differential Equations & Laplace Transforms] --> B[🔄 Transfer Functions<br/>System representation in s-domain]
    B --> C[🎛️ PID Control<br/>Closed-loop control basics]
    C --> D[📊 State-Space Representation<br/>Matrix form for computation]
    D --> E[🎯 LQR Control<br/>Optimal full-state feedback]
    
    style A fill:#e1f5fe
    style E fill:#c8e6c9
```  

---

## 📡 State Estimation Learning Path

**Goal**: Accurately determine rocket position, velocity, orientation using noisy sensor data

```mermaid
flowchart TB
    A[📊 Probability Foundation<br/>Statistics & uncertainty] --> B[🔍 Kalman Filter Theory<br/>Optimal state estimation]
    B --> C[🛠️ Sensor Calibration<br/>Unbiasing & drift correction]
    C --> D[✈️ Flight Mechanics<br/>Rocket dynamics & motion]
    D --> E[🎯 Linearization & Trim<br/>Making nonlinear systems linear]
    
    style A fill:#e1f5fe
    style E fill:#c8e6c9
```  


---

## 🎯 Learning Strategy Recommendations

- **Take detailed notes** - Keep a technical notebook or digital documentation
- **Implement in code** - Don't just watch videos, write and test examples
- **Ask for guidance** - Engage with team leads, but work to figure out answers yourself
- **Start small projects** - Apply knowledge immediately to reinforce learning


### Success Factors
Your learning is a function of how much effort you put into it. Team members who actively engage with leads, take good notes, and implement concepts in code learn a lot faster.