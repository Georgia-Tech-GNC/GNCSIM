
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

### Equations of Motion

Almost all dynamical systems can be written in the form of $\dot{\mathbf{x}}=f(\mathbf{x},\mathbf{u},t)$ and rockets are no exception. This formula is saying that the derivative of the state vector $\dot{\mathbf{x}}$ is a function of the current state $\mathbf{x}$, our control $\mathbf{u}$ and time $t$.

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
* $g = [0, 0, -9.81]^T$ is the gravitational acceleration vector in the inertial frame.
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

Vectors can be computed between the frames using rotation matrix. For example $\mathbf{C}_{BI}$ is the rotation matrix from body to inertial defined above. The rotation matrix from body to component will be defined in a different section. 


### Aerodynamics

These equations can simulate any rigid body with changing mass, however in order to simulate the rigid body dynamics of a rocket, aerodynamics must be taken into account. To do this the forces and moments must be a function of the state. The forces and moments are a function of the altitude $h$, inertial velocity $v_I$, orientation $q$ and angular rate $\omega$. 

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

The first step in computing the forces and moments is getting the velocity in the body frame. To do this a cosine direction matrix is be used defined in a previous section.

$$
\begin{equation}
v_B=\boldsymbol{C}_{IB}(q) (v_I + v_w)
\end{equation}
$$

This formula rotates the velocity in the inertial to the velocity in the body frame. $v_w$ is the velocity of the wind in the inertial frame which can be assumed to be zero if the winds are calm.

Next the velocity must be converted to every componet frame. For the body this rotation matrix is the identity matrix, however for the fin the rotation matrix is defined below where $\delta$ represents the control deflection around the body y axis and $\theta$ represents the rotation angle around the body x axis.

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

Now that the components velocity is known the aerodynamic forces and moments can be calculated. To do this the angle of attack $\alpha$, mach number $M$ and altitude $h$ (in meters) must be known. The formulas for them can be found below with $u$, $v$, and $w$ representing the x, y and z components of velocity and $a$ is the speed of sound, assumed to be 343 m/s.

$$
\begin{equation}
\alpha=\text{atan2}(w,u)
\end{equation}
$$

$$
\begin{equation}
M=\frac{\sqrt{u^2+v^2+w^2}}{a}
\end{equation}
$$

Once $\alpha$, $M$ and $h$ are known this can be used to calculate lift drag and pitching moment. To do this DATCOM is used get the coefficient data from current state of $\alpha$, $M$ and $h$. Getting this lookup table is converted in another section however a 3D linear lookup table is used to get the aerodynamic coefficient.

$$
\begin{equation}
[c_n, c_a, c_m] = \text{aero}(\alpha, M, h)
\end{equation}
$$

The normal coefficient $c_n$, axial coefficient $c_a$ and pitching moment coefficient $c_m$ can be converted to forces and moments in the body frame using the formula below where $S$ is surface area, $c$ is chord length and $q_{\infin}$ is dynamic pressure.

$$
\begin{equation}
N=c_n q_{\infin}S
\end{equation}
$$

$$
\begin{equation}
A=c_a q_{\infin}S
\end{equation}
$$

$$
\begin{equation}
M=c_m q_{\infin}c
\end{equation}
$$

$$
\begin{equation}
q_{\infin} = \frac{1}{2}\rho(h) v^2
\end{equation}
$$

DATCOM does not give lateral forces or roll/yaw moments so it is assumed that they are equal to zero. Therefore the final forces and moments in the component frame are defined below.

$$
\begin{equation}
F_c = [N, 0, A]
\end{equation}
$$

$$
\begin{equation}
M_c = [0, M, 0]
\end{equation}
$$

To convert the forces and moments in the component frame to the body frame a conversion must take place. Recall that $\boldsymbol{C}_{BC} = R_y(\delta)R_x(\theta)$. The inverse can be calculated by taking the transpose.

$$
\begin{equation}
\boldsymbol{C}_{CB} = \boldsymbol{C}_{BC}^T
\end{equation}
$$

From this the forces and moments can be computed using the following formula where $r_{BC} is the distance from the CG to the components reference.

$$
\begin{equation}
F_b = \boldsymbol{C}_{CB}(\delta, \theta)F_c
\end{equation}
$$

$$
\begin{equation}
M_b = \boldsymbol{C}_{CB}(\delta, \theta)M_c + r_{BC} \times F_b
\end{equation}
$$

Once the forces and moments are determined, the process outlined in this section is repeated for all fins and the body. The resulting forces and moments are then summed to obtain the total values, which are then used in the dynamics defined by Equations (1)–(5).


### DATCOM Buildup for Fin

The fins can be defined using five parameters as shown in the MATLAB code below.

```
    fin_params.num_fins = 4;        % [~]
    fin_params.root_chord = 0.203;  % [m]
    fin_params.tip_chord = 0.17;    % [m]
    fin_params.fin_span = 0.144;    % [m]
    fin_params.sweep_deg = 15.0;    % [deg]
```

* *num_fins* is the number of fins attached to the body.
* *root_chord* is the length of the fin’s base (where it attaches to the body).
* *tip_chord* is the length of the fin’s outer edge (furthest from the body).
* *fin_span* is the distance from the body surface to the fin tip.
* *sweep_deg* is the angle of the fin’s leading edge relative to the body axis.

From that the full geometry of the fin can be defined as shown below.

![alt text](fin_info.png)

The parameters above can be formatted into a DATCOM input file as shown below. This file defines the geometry, flow conditions, and analysis setup for a wing-only aerodynamic analysis.

```
 $FLTCON NMACH=3.0, MACH(1)=0.2000,
 0.4000,
 0.6000,
 NALT=3.0, ALT(1)=0.0000,
 5000.0000,
 10000.0000,
 NALPHA=11.0, ALSCHD(1)=-10.0000,
 -8.0000,
 -6.0000,
 -4.0000,
 -2.0000,
 0.0000,
 2.0000,
 4.0000,
 6.0000,
 8.0000,
 10.0000,
 LOOP=2.0$
 $OPTINS SREF=0.0269, CBARR=0.1870, BLREF=0.1440$
 $SYNTHS XW=0.0000, XCG=0.0655$
 $WGPLNF CHRDR=0.2030, CHRDTP=0.1700, SSPN=0.1440, SSPNE=0.1440,
 SAVSI=15.0000, DHDADI=0.0000, TYPE=1.0$
NACA W 4 0012
CASEID GENERATED WING-ONLY ANALYSIS
NEXT CASE
```

#### Flight Conditions

The flight conditions defines the condition grid, also known as the combinations of Mach number, altitude, and angle of attack that DATCOM will compute aerodynamic coefficients for.

```
 $FLTCON NMACH=3.0, MACH(1)=0.2000,
 0.4000,
 0.6000,
 NALT=3.0, ALT(1)=0.0000,
 5000.0000,
 10000.0000,
 NALPHA=11.0, ALSCHD(1)=-10.0000,
 -8.0000,
 -6.0000,
 -4.0000,
 -2.0000,
 0.0000,
 2.0000,
 4.0000,
 6.0000,
 8.0000,
 10.0000,
 LOOP=2.0$
```
* *NMACH=3.0*: There are 3 mach numbers (in this case 0.2, 0.4 and 0.6).
* *NALT=3.0*: There are 3 altitudes (in this case 0, 5km and 10km).
* *NALPHA=11.0*: There are 11 angle of attack in degrees.
* *LOOP=2.0*: DATCOM will loop over Mach and altitude combinations (a grid sweep)
 
It is important to note that the conditions can be chosen from MATLAB and are not hard coded.

#### Reference Dimensions
These are used to nondimensionalize aerodynamic coefficients.

```
 $OPTINS SREF=0.0269, CBARR=0.1870, BLREF=0.1440$
```
* *SREF*: Reference area in $m^2$.
* *CBARR*: Mean aerodynamic chord in $m$.
* *BLREF*: Reference span in $m$.

#### Longitudinal Geometry (CG, Wing Position)
These parameters are used to compute pitching moment and stability derivatives.

```
 $SYNTHS XW=0.0000, XCG=0.0655$
```
* *XW*: Wing apex position from nose in $m$.
* *XCG*: Center of gravity location from nose in $m$.

#### Wing Geometry Definition

These are used to define the platform geometry of the wing.

```
 $WGPLNF CHRDR=0.2030, CHRDTP=0.1700, SSPN=0.1440, SSPNE=0.1440,
 SAVSI=15.0000, DHDADI=0.0000, TYPE=1.0$
```
* *CHRDR*: Root chord in $m$
* *CHRDTP*: Tip chord in $m$
* *SSPN*: Semispan to the break or tip in $m$ (SSPN is assumed to be equal to SSPNE)
* *SSPNE*: Total semispan in $m$
* *SAVSI*: Leading edge sweep angle in degrees
* *DHDADI*: Dihedral angle in degrees (always hard coded to zero)
* *TYPE*: Wing type (1.0 = full-span wing)

#### Airfoil and Other

Defines a NACA 0012 symmetric airfoil and adds labels.

```
NACA W 4 0012
CASEID GENERATED WING-ONLY ANALYSIS
NEXT CASE
```
* *NACA W 0012*: Defines a NACA 0012 symmetric airfoil for wing.
* *CASEID*: A label for the analysis
* *NEXT CASE*: Tells DATCOM that this case is complete and the next one follows

### DATCOM Buildup for Body
TODO

### Control and Trim Points

### Class Structure
TODO

### Simulation
TODO

