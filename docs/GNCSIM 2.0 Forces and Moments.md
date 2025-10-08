
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
\begin{align*}
% Mass
\dot{m} &= f(t) \\[1em]
% Position
\dot{p_I} &= v \\[1em]
% Velocity
\dot{v_I} &= \frac{1}{m}(\mathbf{C}_{BI}(q) F_b) + g \\[1em]
% Quaternion
\dot{q} &= \frac{1}{2} \boldsymbol{\Omega}(q) q  \\[1em]
% Angular Velocity
\dot{\omega} &= \mathbf{I}_b^{-1} (M_b - \omega \times (\mathbf{I}_b \omega)) \\[1em]
\end{align*}
$$

where:
* $f(t)$ is the motor burn rate as function of time.
* $\mathbf{F}_b$ is the sum of external forces in the body frame.
* $\mathbf{M}_b$ is the sum of external moments (torques) in the body frame.
* $\mathbf{g} = [0, 0, -9.81]^T$ is the gravitational acceleration vector in the inertial frame.
* $\mathbf{C}_{IB}$ is the direction cosine matrix that transforms vectors from the inertial frame to the body frame (it is the transpose of $\mathbf{C}_{BI}$).

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

* $\mathbf{I}_b$ is the moment of inertia tensor in the body frame.
* $\boldsymbol{\Omega}$ is the skew-symmetric matrix representation of the angular velocity $\omega$.
$$
\begin{align*}
% Skew Matrix
\boldsymbol{\Omega}(q) &= \begin{bmatrix} 0 & -\omega_x & -\omega_y & -\omega_z \\ \omega_x & 0 & \omega_z & -\omega_y \\ \omega_y & -\omega_z & 0 & \omega_x \\ \omega_z & \omega_y & -\omega_x & 0 \end{bmatrix} 
\end{align*}
$$

