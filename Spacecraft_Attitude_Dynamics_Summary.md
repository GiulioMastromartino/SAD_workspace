# Comprehensive Summary of Course Notes: Spacecraft Attitude Dynamics

**Author/Lecturer:** Prof. Franco Bernelli Zazzera  
**Institution:** Politecnico di Milano  
**Document:** Spacecraft Attitude Dynamics Course Notes (Part 1: Attitude Dynamics and Kinematics)

---

## Overview & Level of Detail Index

This document provides a complete, structured compilation of all topics, arguments, mathematical derivations, and physical interpretations discussed in the course notes.

### Summary Table of Discussed Topics and Level of Detail

| Section / Topic | Level of Detail | Key Concepts & Analytical Depth |
| :--- | :--- | :--- |
| **1. Fundamental Quantities** | High | Inertial & Body frames, Transport Theorem, Angular Momentum derivation, Inertia Matrix & 9 physical inequalities, Rotational Kinetic Energy. |
| **2. Principal Axes & Inertia Ellipsoids** | High | Eigenvalue formulation for principal axes, Inertia Ellipsoid, Kinetic Energy Ellipsoid, Angular Momentum Ellipsoid. |
| **3. Geometrical/Graphical Description of Torque-Free Motion** | Very High | Polhode & Herpolhode, Invariant plane, Intersections of ellipsoids, Planar conic projections on coordinate planes (ellipses vs. hyperbola). |
| **4. Attitude Parameters** | Very High | Direction Cosine Matrix (DCM), Euler Axis/Angle, Quaternions, Gibbs Vector, Euler Angles (all 12 sequences & small angle approximations). |
| **5. Attitude Kinematics** | High | DCM differential equations & orthogonalization, Quaternion differential equations & matrix forms, Gibbs vector kinematics, Euler angle rates & singularities. |
| **6. Euler Equations** | High | Full non-linear coupled dynamical equations, Principal axis simplification, Non-centroidal reference frames & center-of-mass translation coupling. |
| **7. Torque-Free Motion of a Simple Spin Satellite** | Very High | Axisymmetric satellites ($I_x=I_y$), Analytical harmonic solutions, Body cone & Space cone geometry, Euler angles (313 sequence) time evolution. |
| **8. Phase Plane Analysis of Euler Equations** | High | Decoupling via energy & angular momentum integrals, Elliptic/Hyperbolic phase space trajectories ($I_z > I_y > I_x$). |
| **9. Attitude Stability & Linearization** | Medium–High | Lyapunov stability definitions (Lyapunov & Asymptotic), Linearization procedure, Stability of simple spin satellites, Intermediate axis instability (Major/Minor axis rule). |

---

## Detailed Breakdown of Discussed Arguments

---

### 1. Fundamental Quantities

#### 1.1 Inertial and Body-Fixed Frames
* **Inertial Frame ($N = [\underline{i}, \underline{j}, \underline{k}]$):** Orthonormal basis fixed in inertial space. Centre of mass vector denoted by $\underline{R}_C$.
* **Body Frame ($B = [\underline{b}_1, \underline{b}_2, \underline{b}_3]$):** Orthonormal frame fixed to the rigid body with origin at the centre of mass $O$.
* **Attitude Matrix / Direction Cosine Matrix ($A_{B/N}$):**
  $$B = A_{B/N} N$$
  * Properties: $A_{B/N} A_{B/N}^T = I_{3\times 3}$, $\det(A_{B/N}) = 1$.
* **Relative Angular Velocity:**
  $$\underline{\omega}_{B/N} = \underline{\omega}_B - A_{B/N}\underline{\omega}_N$$
  For fixed inertial frame ($\underline{\omega}_N = \underline{0}$), $\underline{\omega}_{B/N} = \underline{\omega}_B = \underline{\omega}$.
* **Transport Theorem:** Relates time derivatives taken in different rotating reference frames:
  $$\left[\frac{d\vec{x}}{dt}\right]_N = \left[\frac{d\vec{x}}{dt}\right]_B + \underline{\omega}_{B/N} \times \vec{x}$$

#### 1.2 Angular Momentum of a Rigid Body
* **Integral Definition:**
  $$\underline{h}_o = \int_B \underline{r} \wedge \underline{v} \, dm$$
  where velocity $\underline{v} = \underline{v}_o + \underline{\omega} \wedge \underline{r} + \underline{v}_{iB}$ (with relative velocity $\underline{v}_{iB} = 0$ for a rigid body).
* **Expanded Vector Expression:**
  $$\underline{h}_o = -\underline{v}_o \wedge S_o + I\underline{\omega}$$
  where $S_o = \int_B \underline{r} \, dm$ is the first moment of mass.
* **Inertia Matrix ($I$):**
  $$I = \begin{bmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{yx} & I_{yy} & I_{yz} \\ I_{zx} & I_{zy} & I_{zz} \end{bmatrix}$$
  * *Inertia Moments (Diagonal):* $I_{xx} = \int_B (y^2 + z^2) dm$, $I_{yy} = \int_B (x^2 + z^2) dm$, $I_{zz} = \int_B (x^2 + y^2) dm$ (Always positive).
  * *Inertia Products (Off-diagonal):* $I_{xy} = \int_B -xy \, dm$, etc.

#### 1.3 Constraints on Inertia Matrix Elements (9 Physical Inequalities)
* **6 Triangular Inequalities on Moments:**
  $$I_{xx} + I_{yy} \ge I_{zz}, \quad |I_{xx} - I_{yy}| \le I_{zz} \quad (\text{plus cyclic permutations})$$
* **3 Inequalities on Products of Inertia:**
  $$I_{xx} \ge 2 I_{zy}, \quad I_{yy} \ge 2 I_{xz}, \quad I_{zz} \ge 2 I_{xy}$$

#### 1.4 Rotational Kinetic Energy
* **Definition:**
  $$2T_{rot} = 2\underline{v}_o \cdot (\underline{\omega} \wedge S_o) + \underline{\omega} \cdot I \underline{\omega}$$
* **Center of Mass Origin ($S_o = 0$):**
  $$\underline{h}_o = I \underline{\omega}, \quad 2T = \underline{\omega} \cdot I \cdot \underline{\omega}$$

---

### 2. Principal Axes and Inertia Ellipsoids

#### 2.1 Principal Axes Transformation
* Solving the matrix eigenvalue problem $a I a^T = \text{diag}(I_x, I_y, I_z)$ diagonalizes the inertia matrix, where $a$ consists of eigenvectors representing direction cosines of the principal inertia frame.
* **Simplified Scalar Forms in Principal Axes:**
  $$\underline{h} = \begin{bmatrix} I_x \omega_x \\ I_y \omega_y \\ I_z \omega_z \end{bmatrix}, \quad 2T = I_x \omega_x^2 + I_y \omega_y^2 + I_z \omega_z^2$$
* Symmetry planes contain 2 principal axes; the 3rd axis is perpendicular. The principal inertia set always includes the maximum and minimum possible moments of inertia.

#### 2.2 Inertia Ellipsoid
* Formed by expressing directional inertia $I_\eta$ along unit vector $\hat{\eta}$:
  $$\frac{l_{\eta x}^2 / I_\eta}{1/I_x} + \frac{l_{\eta y}^2 / I_\eta}{1/I_y} + \frac{l_{\eta z}^2 / I_\eta}{1/I_z} = 1$$
* **Semi-axes:** $\sqrt{1/I_x}, \sqrt{1/I_y}, \sqrt{1/I_z}$.

#### 2.3 Kinetic Energy Ellipsoid
* Trajectory of feasible $\underline{\omega}$ for a given kinetic energy $T$:
  $$\frac{\omega_x^2}{2T/I_x} + \frac{\omega_y^2}{2T/I_y} + \frac{\omega_z^2}{2T/I_z} = 1$$

#### 2.4 Angular Momentum Ellipsoid
* Trajectory of feasible $\underline{\omega}$ for constant magnitude of angular momentum $h$:
  $$\frac{\omega_x^2}{h^2/I_x^2} + \frac{\omega_y^2}{h^2/I_y^2} + \frac{\omega_z^2}{h^2/I_z^2} = 1$$

---

### 3. Geometrical/Graphical Description of Torque-Free Motion

#### 3.1 Polhodes
* Defined as the curve/line of intersection between the Kinetic Energy Ellipsoid and the Angular Momentum Ellipsoid in body-fixed space.
* **Existence Condition for Real Rotational Motion ($I_x > I_y > I_z$):**
  $$I_x > \frac{h^2}{2T} > I_z$$

#### 3.2 Invariant Plane and Herpolhode
* **Invariant Plane:** Fixed plane in inertial space orthogonal to constant angular momentum vector $\underline{h}$, at distance $\omega_h = \frac{\underline{\omega}\cdot\underline{h}}{|\underline{h}|} = \frac{2T}{h} = \text{const}$.
* **Herpolhode:** Non-closed curve traced by the tip of the angular velocity vector $\underline{\omega}$ on the invariant plane as the kinetic energy ellipsoid rolls on it.

#### 3.3 Planar Conic Projections of Polhodes
* **$(x-y)$ Plane Projection:** Ellipse ($\omega_x^2 A + \omega_y^2 B = 1$).
* **$(y-z)$ Plane Projection:** Ellipse.
* **$(x-z)$ Plane Projection:** **Hyperbola** (demonstrating geometric instability around intermediate axis $y$).

---

### 4. Attitude Parameters

#### 4.1 Direction Cosines Matrix (DCM) - 9 Parameters (6 Constraints)
* Maps vectors between frames: $\underline{a}_B = A_{B/N} \underline{a}_N$.
* Orthogonality properties: $A A^T = I$, $A^T = A^{-1}$.
* **Successive Rotations:** $A'' = A' A$ (Multiplication in reverse chronological order).

#### 4.2 Euler Axis and Angle ($\underline{e}, \phi$) - 4 Parameters (1 Constraint)
* Based on Euler's Rotation Theorem: real orthogonal matrix has one unit eigenvalue with eigenvector $\underline{e}$.
* **Relations:**
  $$\cos \phi = \frac{1}{2}(\text{tr}(A) - 1), \quad e_1 = \frac{A_{23}-A_{32}}{2\sin\phi}, \quad e_2 = \frac{A_{31}-A_{13}}{2\sin\phi}, \quad e_3 = \frac{A_{12}-A_{21}}{2\sin\phi}$$
* **Singularity:** Undefined Euler axis when $\sin\phi = 0$ ($\phi = n\pi$).

#### 4.3 Quaternions ($\underline{q}, q_4$) - 4 Parameters (1 Constraint)
* **Definition:** $q_i = e_i \sin(\phi/2)$ for $i=1,2,3$, and $q_4 = \cos(\phi/2)$, satisfying $q_1^2 + q_2^2 + q_3^2 + q_4^2 = 1$.
* **Advantages:** Complete singularity-free parameterization.
* **Quaternion Multiplication (Rotation Composition):** $q'' = q' \otimes q$.

#### 4.4 Gibbs Vector ($\underline{g}$) - 3 Parameters (Minimal)
* **Definition:** $g_i = e_i \tan(\phi/2) = q_i / q_4$.
* **Composition Rule:**
  $$g'' = \frac{\underline{g} + \underline{g}' - \underline{g}' \wedge \underline{g}}{1 - \underline{g} \cdot \underline{g}'}$$
* **Singularity:** Singular when $\phi = (2n+1)\pi$.

#### 4.5 Euler Angles - 3 Parameters (Minimal)
* Sequence of 3 successive rotations about local coordinate axes.
* **12 Possible Sequences:**
  * Symmetric/Symmetric-like (first and third index equal): `121, 131, 212, 232, 313, 323` (Singularity at $\theta = n\pi$).
  * Asymmetric (all indices different): `123, 132, 213, 231, 312, 321` (Singularity at $\theta = (2n+1)\pi/2$).
* **Small Angle Approximations:** Sequence order independence, $A \approx I - [\text{angles}\wedge]$.

---

### 5. Attitude Kinematics

#### 5.1 DCM Kinematics
* **Differential Equation:**
  $$\frac{dA}{dt} = -[\underline{\omega} \wedge] A$$
* **Numerical Orthogonalization:** Iterative update formula $A_{k+1} = \frac{3}{2}A_k - \frac{1}{2}A_k A_k^T A_k$.

#### 5.2 Quaternion Kinematics
* **Differential Equation Matrix Form:**
  $$\frac{dq}{dt} = \frac{1}{2} \Omega q, \quad \text{where } \Omega = \begin{bmatrix} 0 & \omega_3 & -\omega_2 & \omega_1 \\ -\omega_3 & 0 & \omega_1 & \omega_2 \\ \omega_2 & -\omega_1 & 0 & \omega_3 \\ -\omega_1 & -\omega_2 & -\omega_3 & 0 \end{bmatrix}$$

#### 5.3 Gibbs Vector Kinematics
* **Differential Equation:**
  $$\frac{d\underline{g}}{dt} = \frac{1}{2} \left[ \underline{\omega} - \underline{\omega} \wedge \underline{g} + (\underline{g} \cdot \underline{\omega})\underline{g} \right]$$

#### 5.4 Euler Angle Kinematics
* Derivation via addition of angular velocity vectors along moving axes.
* Includes explicit sets of differential equations for all 12 sequences.
* **Non-Integrability of Angular Velocity:** Integrability of Euler angles proven, contrasting with angular velocities ($\omega_i \, dt$ is not an exact differential).

---

### 6. Euler Equations of Motion

#### 6.1 General Vector & Non-Principal Axis Dynamics
* **Vector Form:** $\left[\frac{d\underline{h}}{dt}\right]_B + \underline{\omega} \wedge \underline{h} = \underline{M}$.
* **General Matrix Form:** Strongly coupled 1st order non-linear system containing inertia products.

#### 6.2 Principal Axis Simplified Euler Equations
$$\begin{cases} I_x \dot{\omega}_x + (I_z - I_y)\omega_z \omega_y = M_x \\ I_y \dot{\omega}_y + (I_x - I_z)\omega_x \omega_z = M_y \\ I_z \dot{\omega}_z + (I_y - I_x)\omega_y \omega_x = M_z \end{cases}$$

#### 6.3 Equations referred to Non-Centroidal Frame Origin
* Additional torque component due to center-of-mass linear acceleration $\underline{a}_A$:
  $$\underline{M}_A = \frac{d\underline{h}_A}{dt} + \underline{S}_A \wedge \underline{a}_A$$

---

### 7. Torque-Free Motion of a Simple Spin Satellite

#### 7.1 Analytical Solution for Axial Symmetry ($I_x = I_y$)
* $\omega_z = \text{const} = \omega_{z0}$.
* Transverse angular velocity magnitude $\omega_{xy} = \sqrt{\omega_x^2 + \omega_y^2} = \text{const}$.
* **Precession Frequency ($\lambda$):**
  $$\lambda = \frac{(I_z - I_x)\omega_z}{I_x}$$
* Harmonic solutions for transverse components:
  $$\omega_x(t) = \omega_{x0}\cos(\lambda t) - \omega_{y0}\sin(\lambda t), \quad \omega_y(t) = \omega_{x0}\sin(\lambda t) + \omega_{y0}\cos(\lambda t)$$

#### 7.2 Geometric Cones (Body Cone vs. Space Cone)
* **Nutation Angle ($\theta$) and Angular Velocity Angle ($\gamma$):**
  $$\tan\gamma = \frac{\omega_{xy}}{\omega_z}, \quad \tan\theta = \frac{I_x}{I_z} \tan\gamma$$
* **Major-Axis Spin ($I_z > I_x$):** $\lambda > 0$, $\theta < \gamma$, body cone rolls inside space cone.
* **Minor-Axis Spin ($I_z < I_x$):** $\lambda < 0$, $\theta > \gamma$, body cone rolls outside space cone.

#### 7.3 Time Evolution of 313 Euler Angles
$$\phi(t) = \phi_0 + \dot{\phi}t, \quad \psi(t) = \psi_0 + \dot{\psi}t, \quad \theta(t) = \theta_0 = \text{const}$$
where $\dot{\psi} = -\lambda$, $\dot{\phi} = \frac{\omega_z + \lambda}{\cos\theta_0}$.

---

### 8. Phase Plane Analysis of Euler Equations

#### 8.1 Decoupled Phase Space Differential Equations
* By combining kinetic energy $2T$ and angular momentum $h^2$ integrals, second-order decoupled equations are obtained:
  $$\ddot{\omega}_i + P_i \omega_i + Q_i \omega_i^3 = 0, \quad (i = x, y, z)$$
* **First Integral Phase Plane Conic Sections:**
  $$\dot{\omega}_i^2 + P_i \omega_i^2 + \frac{1}{2} Q_i \omega_i^4 = K_i$$

#### 8.2 Qualitative Behavior ($I_z > I_y > I_x$)
* $x$ and $z$ phase planes: Trajectories remain bounded closed ellipses.
* $y$ phase plane (intermediate axis): $Q_y < 0$, creating hyperbolic paths that demonstrate open, divergent trajectories near origin (instability).

---

### 9. Attitude Stability and Linearization

#### 9.1 Mathematical Definitions of Stability
* **Lyapunov Stability:** Bounded state response for bounded initial perturbations.
* **Asymptotic Stability:** Lyapunov stable + state convergence to origin as $t \to \infty$.
* **Lyapunov's First Stability Theorem:** Preservation of stability characteristics between original non-linear system and its Jacobian linearization at equilibrium points.

#### 9.2 Stability Analysis of Torque-Free Simple Spin Satellite
* Perturbation around equilibrium $\underline{\bar{\omega}} = [0, 0, \bar{\omega}_z]^T$.
* **Characteristic Equation Eigenvalue Parameter:**
  $$\lambda^2 = \frac{(I_z - I_y)(I_z - I_x)}{I_x I_y} \bar{\omega}_z^2$$
* **Stability Condition ($\lambda^2 > 0$):**
  * Spin about **Major Principal Axis** ($I_z > I_y$ and $I_z > I_x$): **STABLE**.
  * Spin about **Minor Principal Axis** ($I_z < I_y$ and $I_z < I_x$): **STABLE** (rigid body assumption).
  * Spin about **Intermediate Principal Axis** ($I_x < I_z < I_y$ or $I_y < I_z < I_x$): **UNSTABLE**.
