
# Rocket Dynamics for GNCSIM 2.0

### State Vector Definition

The state vector is all the necessary information to describe the state of the system. For GNCSIM 2.0, the state vector includes mass, position in the inertial frame, velocity in the inertial frame, quaternion from inertial to body, and angular rate expressed in the body frame.

$$
\mathbf{x} = 
\begin{bmatrix}
    m \\
    p_I \\
    v_I \\
    q_{IB} \\
    \omega_B
\end{bmatrix}
$$

where:
* $m$ is the mass of the body.
* $p_I = [x, y, z]^T$ is the position vector in the inertial frame.
* $v_I = [v_x, v_y, v_z]^T$ is the velocity vector in the inertial frame.
* $q_{IB} = [q_0, q_1, q_2, q_3]^T$ is the quaternion representing the orientation from the inertial frame to the body frame.
* $\omega = [\omega_x, \omega_y, \omega_z]^T$ is the angular velocity vector in the body frame.

The total number of states in the system is 14, which includes 1 for mass, 3 for position, velocity and angular rate and 4 for quaternion.

---
### Equations of Motion (no control)

Almost all dynamical systems can be written in the form of $\dot{\mathbf{x}}=f(\mathbf{x},\mathbf{u},t)$ and rockets are no exeption. This formula is saying that the derivatave of the state vector $\dot{\mathbf{x}}$ is a function of the current state $\mathbf{x}$, our control $\mathbf{u}$ and time $t$.

The time derivative of the state vector, $\dot{\mathbf{x}}$, is given by the following equations:


$$
% Mass
\begin{equation}
\dot{m} = f(t)
\end{equation}
$$
$$
% Position
\begin{equation}
\dot{p}_I = v_I
\end{equation}
$$
$$
% Velocity
\begin{equation}
\dot{v}_I = \frac{1}{m} \left( \mathbf{C}_{BI}(q) F_b \right) + g
\end{equation}
$$
$$
% Quaternion
\begin{equation}
\dot{q} = \frac{1}{2} \boldsymbol{\Omega}(\omega) q
\end{equation}
$$
$$
% Angular Velocity
\begin{equation}
\dot{\omega} = \mathbf{I}_b^{-1} \left(M_b - \omega \times (\mathbf{I}_b \omega) \right)
\end{equation}
$$



where:
* $f(t)$ is the motor burn rate as function of time.
* $F_b$ is the sum of external forces in the body frame.
* $M_b$ is the sum of external moments (torques) in the body frame.
* $\mathbf{g} = [0, 0, -9.81]^T$ is the gravitational acceleration vector in the inertial frame.
* $\mathbf{C}_{IB}$ is the direction cosine matrix that transforms vectors from the inertial frame to the body frame (it is the transpose of $\mathbf{C}_{BI}$). $\mathbf{C}_{BI}=\mathbf{C}_{IB}^T$

$$
\begin{equation}
C_{IB}(q) =
\begin{bmatrix}
1 - 2(q_2^2 + q_3^2) & 2(q_1 q_2 + q_0 q_3) & 2(q_1 q_3 - q_0 q_2) \\
2(q_1 q_2 - q_0 q_3) & 1 - 2(q_1^2 + q_3^2) & 2(q_2 q_3 + q_0 q_1) \\
2(q_1 q_3 + q_0 q_2) & 2(q_2 q_3 - q_0 q_1) & 1 - 2(q_1^2 + q_2^2)
\end{bmatrix}
\end{equation}
$$

* $\mathbf{I}_b$ is the moment of inertia tensor in the body frame. Note that if the moment of inertia is dependt on the mass and CG location.
* $\boldsymbol{\Omega}(w)$ is the skew-symmetric matrix representation of the angular velocity $\omega$.
$$
\begin{equation}
\boldsymbol{\Omega}(q) = \begin{bmatrix} 0 & -\omega_x & -\omega_y & -\omega_z \\ \omega_x & 0 & \omega_z & -\omega_y \\ \omega_y & -\omega_z & 0 & \omega_x \\ \omega_z & \omega_y & -\omega_x & 0 \end{bmatrix} 
\end{equation}
$$

### Coordinate Frames

There are three major frames used in calculating forces and moments. The first is the **inertial frame**, which (as the name implies) is inertial, with the x-axis facing east, the y-axis facing north, and the z-axis facing up. This frame is also known as **ENU (East, North, Up)**.

The next frame is the **body frame**, with the origin at the nose tip and the x-axis facing toward the tail. The y and z axes are defined arbitrarily.

Finally, the **component frame** is defined the same as the body frame; however, the origin is located at the tip of each component.

Vectors can be computed between the frames using rotation matices. For example $\mathbf{C}_{BI}$ is the rotation matrix from body to inertial defined above. The rotation matrix from body to component will be defined in a different section. 


### Aerodynamics

These equations can simulate any rigid body with changing mass, however in order to simulate the rigid body dynamics of a rocket, aerodynamics must be taken into account. To do this the forces and moments must be a function of the state. The forces and moments are a function of the altitdue $h$, iniertal velocity, orientation and angular rate. 

$$
\begin{equation}
F_b=f(h, v_I, q, \omega)
\end{equation}
$$

$$
\begin{equation}
M_b=f(h, v_I, q, \omega)
\end{equation}
$$

The first step in computing the forces and moments is getting the velocity in the body frame. To do this the cosine direction matrix can be used.

$$
\begin{equation}
v_B=\boldsymbol{C}_{IB}(q) (v_I + v_w)
\end{equation}
$$

This formula rotates the velocity in the inertial to the velocity in the body frame. $v_w$ is the velocity of the wind in the inertial frame which can be assumed to be zero if the winds are calm.

Next the velocity must be converted to every componet frame. For the body this rotation matrix is the idenity matrix, however for the fin the rotation matrix is defined below where $\delta$ represents the control deflection around the body y axis and $\theta$ represents the rotation angle around the body x axis.

$$
\begin{equation}
\boldsymbol{C}_{BC} = R_y(\delta)R_x(\theta)
\end{equation}
$$

$$
\begin{equation}
\boldsymbol{C}_{BC}(\delta, \theta) = 
\begin{bmatrix}
\cos\delta & 0 & -\sin\delta \\
0 & 1 & 0 \\
\sin\delta & 0 & \cos\delta
\end{bmatrix}
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\theta & \sin\theta \\
0 & -\sin\theta & \cos\theta
\end{bmatrix}
\end{equation}
$$

Assuming there are four fins each with zero control deflection $\delta$, the rotation matrix for each fin would be when $\delta=0^\circ$ and $\theta=0^\circ$, $\theta=90^\circ$, $\theta=180^\circ$, and $\theta=270^\circ$. It is important to note that the rotation around the x-axis $\theta$ remains static as it represents the placement of the fins, whereas the control deflection $\delta$ may be dynamic as it represents control deflection of the fin.

From this, the component velocity and angular rate can be found using the formula below.

$$
\begin{equation}
v_C = \boldsymbol{C}_{BC}(\delta, \theta)(v_B + \omega \times r_{BC})
\end{equation}
$$

$$
\begin{equation}
w_C = \boldsymbol{C}_{BC}(\delta, \theta)w_B
\end{equation}
$$

$r_{BC}$ represents the translation vector from the center of mass to the component reference location. $v_C$ and $\omega_C$ represent the velocity and angular rate of the fin.

Now that the componet velocity is known the aerodynamic forces and moments can be calculated. To do this the angle of attack, side slip and mach number must be known. The formulas for them can be found below with $u$, $v$, and $w$ representing the x, y and z componet of velocity and $a$ is the speed of sound, assumed to be 343 m/s.

$$
\begin{equation}
M=\frac{\sqrt{u^2+v^2+w^2}}{a}
\end{equation}
$$
$$
\begin{equation}
\alpha=\text{atan2}(w,u)
\end{equation}
$$
$$
\begin{equation}
\beta=\text{asin}\left( \frac{v}{\sqrt{u^2+v^2+w^2}} \right)
\end{equation}
$$

Once $\alpha$, $\beta$ and $M$ are known this can be used to calcuale lift drag and pitching moment. To do this DATCOM is used get the coeffient data from current state of $\alpha$, $\beta$ and $M$.

$$
\begin{equation}
[c_l, c_d, c_m] = \text{aero}(\alpha, \beta, M)
\end{equation}
$$

### DATCOM Buildup for Fin


### DATCOM Buildup for Body


### Fin Example with DATCOM


### Body Example with DATCOM

### Simulation


