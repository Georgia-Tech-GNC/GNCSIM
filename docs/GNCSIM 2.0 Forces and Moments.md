
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
* $q_{IB} = [q_w, q_x, q_y, q_z]^T$ is the quaternion representing the orientation from the inertial frame to the body frame.
* $\omega = [\omega_x, \omega_y, \omega_z]^T$ is the angular velocity vector in the body frame.

The total number of states in the system is 14, which includes 1 for mass, 3 for position, velocity and angular rate and 4 for quaternion.

---
### Equations of Motion

Any 

The time derivative of the state vector, $\dot{\mathbf{x}}$, is given by the following equations:

$$
\begin{align*}
% Mass
\dot{m} &= 0 \\[1em]
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
* $\mathbf{F}_b$ is the sum of external forces in the body frame.
* $\mathbf{M}_b$ is the sum of external moments (torques) in the body frame.
* $\mathbf{g} = [0, 0, -9.81]^T$ is the gravitational acceleration vector in the inertial frame.
* $\mathbf{C}_{I_B}$ is the direction cosine matrix that transforms vectors from the body frame to the inertial frame (it is the transpose of $\mathbf{C}_{B_I}$).
* $\mathbf{I}_b$ is the moment of inertia tensor in the body frame.
* $\boldsymbol{\Omega}$ is the skew-symmetric matrix representation of the angular velocity $\omega$.
$$
\begin{align*}
% Skew Matrix
\boldsymbol{\Omega}(q) &= \begin{bmatrix} 0 & -\omega_x & -\omega_y & -\omega_z \\ \omega_x & 0 & \omega_z & -\omega_y \\ \omega_y & -\omega_z & 0 & \omega_x \\ \omega_z & \omega_y & -\omega_x & 0 \end{bmatrix} \begin{bmatrix} q_w \\ q_x \\ q_y \\ q_z \end{bmatrix}
\end{align*}
$$

