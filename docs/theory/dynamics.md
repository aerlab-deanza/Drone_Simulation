# Rigid-body dynamics and RK4

:::{admonition} In plain words
:class: aerl-plain
The simulator treats the drone as one rigid object pushed by four propellers.
Two laws of physics move it: **Newton's law** (forces change its velocity) and
**Euler's rotation law** (torques change its spin). The four motor speeds are
converted into one total thrust and three turning torques. A numerical method
called **RK4** then steps the motion forward in time, accurately enough that
halving the time step makes the error 16 times smaller.
:::

```{localvideo} theory/v4_control.mp4
:title: Dynamics and control: RK4, PID and LQR
```

## The key idea

{class}`~drone_sim.dynamics.plant.DronePlant` tracks 12 numbers: position,
velocity, the three Euler angles, and the three body spin rates. Each time
step uses two equations:

$$
\underbrace{\dot{\mathbf v} = \frac{1}{m}\big(R\,[0, 0, -T]^\top - k_d\,\mathbf v\big) + [0, 0, g]^\top}_{\text{Newton: forces}}
\qquad
\underbrace{\boldsymbol\tau = I\dot{\boldsymbol\omega} + \boldsymbol\omega \times (I\boldsymbol\omega)}_{\text{Euler: torques}}
$$

The four motor speeds $\omega_1 \dots \omega_4$ set the thrust and torques
through one fixed **mixing matrix** $M$:
$[T, \tau_x, \tau_y, \tau_z]^\top = M\,[\omega_1^2, \omega_2^2, \omega_3^2, \omega_4^2]^\top$.

| Symbol | Meaning |
|---|---|
| $m$, $I$ | mass (kg) and moment of inertia (kg·m²) |
| $T$ | total thrust of the four propellers (N) |
| $\boldsymbol\tau$ | turning torques about the body axes (N·m) |
| $R$ | body-to-world rotation (points the thrust) |
| $k_d$ | air drag coefficient |
| $\boldsymbol\omega$ | body spin rate (rad/s) |

The $\boldsymbol\omega \times (I\boldsymbol\omega)$ term is the **gyroscopic
effect**: spinning about one axis while pitching causes a roll. RK4 advances
all 12 numbers by sampling the slope four times per step and averaging.

## Show the derivations

::::{dropdown} Euler's rotation equations, component by component
:icon: unfold

In body axes, the angular momentum is $\mathbf L = I\boldsymbol\omega$. Because
the body frame rotates, the derivative of $\mathbf L$ in the world equals its
body-frame derivative plus a transport term:

$$
\boldsymbol\tau = I\dot{\boldsymbol\omega} + \boldsymbol\omega \times (I\boldsymbol\omega).
$$

For a symmetric quadcopter $I = \operatorname{diag}(I_{xx}, I_{yy}, I_{zz})$, and
the components are

$$
\dot\omega_x = \frac{(I_{yy}-I_{zz})\,\omega_y\omega_z + \tau_x}{I_{xx}},\quad
\dot\omega_y = \frac{(I_{zz}-I_{xx})\,\omega_x\omega_z + \tau_y}{I_{yy}},\quad
\dot\omega_z = \frac{(I_{xx}-I_{yy})\,\omega_x\omega_y + \tau_z}{I_{zz}}.
$$

The attitude itself evolves through the Euler rate matrix $E$ from
{doc}`euler-angles`: $[\dot\phi, \dot\theta, \dot\psi]^\top = E(\phi,\theta)\,\boldsymbol\omega$.
Translation is derived in {doc}`frames`.
::::

::::{dropdown} Derivation: the motor mixing matrix M
:icon: unfold

Each motor $i$ spins at $\omega_i$ and produces thrust $f_i = k_t\omega_i^2$
along body $-z$, plus a drag torque $k_b\omega_i^2$ that opposes its spin.
In an X frame the motors sit at $45^\circ$, so each one is
$a = \ell/\sqrt2$ from both body axes, where $\ell$ is the arm length.

The torque of a force $\mathbf F = [0, 0, -f]$ at $\mathbf r = [r_x, r_y, 0]$ is
$\mathbf r \times \mathbf F = [-r_y f,\; r_x f,\; 0]$. With motor 1 front-left
$(a, -a)$, 2 front-right $(a, a)$, 3 rear-right $(-a, a)$, and 4 rear-left
$(-a, -a)$ in FRD:

$$
\begin{bmatrix} T\\ \tau_x\\ \tau_y\\ \tau_z \end{bmatrix} =
\underbrace{\begin{bmatrix}
k_t & k_t & k_t & k_t\\
a k_t & -a k_t & -a k_t & a k_t\\
a k_t & a k_t & -a k_t & -a k_t\\
-k_b & k_b & -k_b & k_b
\end{bmatrix}}_{M}
\begin{bmatrix} \omega_1^2\\ \omega_2^2\\ \omega_3^2\\ \omega_4^2 \end{bmatrix}.
$$

Motors 1 and 3 spin clockwise seen from above, so their reaction torque is
counter-clockwise from above, which is negative about the downward $z$ axis.
::::

::::{dropdown} Derivation: what happens when the motors saturate
:icon: unfold

$M$ is invertible, so any $(T, \boldsymbol\tau)$ has a unique set of squared
speeds ({meth}`~drone_sim.dynamics.plant.DronePlant.inverse_mixing`). But
squared speeds must lie in $[0, \omega_{\max}^2]$.
{meth}`~drone_sim.dynamics.plant.DronePlant.allocate_control` handles
saturation by writing the solution as
$\boldsymbol s = \boldsymbol s_T + \lambda\,\boldsymbol s_\tau$ (collective part plus
torque part). It then picks the largest $\lambda \in [0, 1]$ that keeps every
motor feasible:

$$
\lambda = \min_i \begin{cases}
(\omega_{\max}^2 - s_{T,i})/s_{\tau,i} & s_{\tau,i} > 0\\
s_{T,i}/(-s_{\tau,i}) & s_{\tau,i} < 0
\end{cases}
$$

This keeps the thrust and the *direction* of the torque, and gives up only
torque magnitude.
::::

::::{dropdown} Derivation: RK4 and its accuracy
:icon: unfold

The plant is advanced with the classical fourth-order Runge-Kutta method
({func}`~drone_sim.dynamics.plant.rk4_step`), holding $T$ and
$\boldsymbol\tau$ constant over the step (a zero-order hold, like a digital
controller):

$$
\begin{aligned}
k_1 &= f(t, \mathbf x), &
k_2 &= f(t + \tfrac h2, \mathbf x + \tfrac h2 k_1),\\
k_3 &= f(t + \tfrac h2, \mathbf x + \tfrac h2 k_2), &
k_4 &= f(t + h, \mathbf x + h k_3),
\end{aligned}
\qquad
\mathbf x_{n+1} = \mathbf x_n + \tfrac h6 (k_1 + 2k_2 + 2k_3 + k_4).
$$

Matching the Taylor series of the exact solution shows that the local error is
$O(h^5)$, so the error over a fixed time span is $O(h^4)$. Halving the step
cuts the error 16-fold. The forward Euler method's global error is only $O(h)$.
For rotation it also steadily adds energy: a spinning body integrated with
Euler speeds up.

The test `test_rk4_integrates_angular_acceleration_and_rate` checks RK4
against the closed-form answer for a constant torque:
$\omega = \alpha t$, $\psi = \tfrac12 \alpha t^2$ (exact, since RK4 integrates
polynomials up to degree 4 exactly). The notebook
{doc}`/examples/notebooks/rk4_testing` explores the plant response further.
::::

:::{admonition} Why it matters for the drone
:class: aerl-why
- On the first step of the demo, the controller asks for about **10× more
  torque** than the motors can deliver. Thanks to the saturation logic, the
  drone keeps its thrust and simply turns more slowly (finding 10).
- Every RK4 stage must use the full trial state. An early version reused the
  current spin rates in every stage, so it wasn't really RK4 (finding 12).
:::
