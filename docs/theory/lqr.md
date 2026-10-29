# Linear-quadratic regulator

:::{admonition} In plain words
:class: aerl-plain
Instead of tuning three PID loops by hand, the **linear-quadratic regulator
(LQR)** computes the whole controller at once. You state what you care about:
how much each kind of error costs, and how much each kind of motor effort
costs. The maths then returns the single best set of gains for that
trade-off. The catch is that "best" assumes the drone stays close to hover.
Far from hover the LQR still works, but it is no longer optimal.
:::

```{localvideo} theory/v4_control.mp4
:title: Dynamics and control: RK4, PID and LQR
```

## The key idea

Near hover, the drone's dynamics are approximately **linear**:
$\dot{\mathbf x} \approx A\mathbf x + B\mathbf u$. The LQR picks the control
$\mathbf u$ that minimizes a total cost, summed over all future time:

$$
J = \int_0^\infty \big(\underbrace{\mathbf x^\top Q\,\mathbf x}_{\text{error}} +
\underbrace{\mathbf u^\top R\,\mathbf u}_{\text{effort}}\big)\,dt .
$$

The answer is a fixed matrix of gains, applied to the error at every instant:

$$
\mathbf u = \mathbf u_{\text{hover}} - K\,(\mathbf x - \mathbf x_{\text{target}}),
\qquad K = R^{-1}B^\top P,
$$

where $P$ comes from solving the **Riccati equation**, which the project
solves from scratch.

| Symbol | Meaning |
|---|---|
| $\mathbf x$ | the 12-number state (position, velocity, angles, spin rates) |
| $\mathbf u$ | the four inputs: thrust and three torques |
| $A, B$ | the linearized dynamics near hover |
| $Q$ | how much each state error costs (big = care more) |
| $R$ | how much each input costs (big = use it gently) |
| $K$ | the resulting gain matrix (4×12) |
| $P$ | solution of the Riccati equation; $\mathbf x^\top P\mathbf x$ is the best achievable cost |

In code: {class}`~drone_sim.control.lqr.LQRController`.

## Show the derivations

::::{dropdown} Step 1: linearizing about hover
:icon: unfold

At hover, $\mathbf x_{eq} = 0$ and $\mathbf u_{eq} = [mg, 0, 0, 0]$. For small
deviations $\delta\mathbf x$, $\delta\mathbf u$,

$$
\delta\dot{\mathbf x} \approx A\,\delta\mathbf x + B\,\delta\mathbf u,\qquad
A = \left.\frac{\partial f}{\partial \mathbf x}\right|_{eq},\quad
B = \left.\frac{\partial f}{\partial \mathbf u}\right|_{eq}.
$$

{func}`~drone_sim.control.lqr.linearize` computes them with central
differences, $\big(f(\mathbf x + \varepsilon\mathbf e_i) - f(\mathbf x - \varepsilon\mathbf e_i)\big)/2\varepsilon$,
which has error $O(\varepsilon^2)$.

The structure is easy to see by hand. At hover the thrust direction is
$R\,[0,0,-T]^\top$; to first order in the angles this gives

$$
\ddot x \approx -g\,\theta, \qquad \ddot y \approx g\,\phi, \qquad
\ddot z = -\delta T/m, \qquad \dot{\boldsymbol\omega} = I^{-1}\boldsymbol\tau .
$$

So the linear model is four chains of integrators: torque → rate → angle →
(×g) acceleration → velocity → position. That is why position can only be
controlled *through* attitude.

The default weights $Q = \operatorname{diag}(10,10,10,\,1,1,1,\,50,50,20,\,1,1,1)$
and $R = \operatorname{diag}(0.5, 20, 20, 20)$ favour level attitude and allow
generous thrust while keeping torques gentle.
::::

::::{dropdown} Step 2: deriving the Riccati equation (Hamilton–Jacobi–Bellman)
:icon: unfold

Guess that the optimal cost-to-go is quadratic, $V(\mathbf x) = \mathbf x^\top P\mathbf x$
with $P = P^\top$. The Hamilton–Jacobi–Bellman equation for an infinite
horizon says

$$
0 = \min_{\mathbf u}\Big[\mathbf x^\top Q\mathbf x + \mathbf u^\top R\mathbf u
+ \nabla V^\top (A\mathbf x + B\mathbf u)\Big],
\qquad \nabla V = 2P\mathbf x .
$$

The bracket is a convex quadratic in $\mathbf u$. Setting its gradient to zero,
$2R\mathbf u + 2B^\top P\mathbf x = 0$, gives the optimal control

$$
\mathbf u^\star = -K\mathbf x, \qquad K = R^{-1}B^\top P .
$$

Substituting back, the condition must hold for every $\mathbf x$, so the
symmetric matrix in the middle must vanish:

$$
\boxed{A^\top P + P A - P B R^{-1} B^\top P + Q = 0}
$$

This is the continuous-time **algebraic Riccati equation** (CARE).

**Stability.** With $\mathbf u^\star$, $\dot V = -\mathbf x^\top(Q + K^\top R K)\mathbf x \le 0$,
so $V$ is a Lyapunov function and the closed loop
$A - BK$ is stable whenever $(A, B)$ is stabilizable and $(A, Q^{1/2})$ is
detectable. {meth}`~drone_sim.control.lqr.LQRController.closed_loop_eigenvalues`
checks this numerically: every eigenvalue has a negative real part.
::::

::::{dropdown} Step 3: solving it with the Hamiltonian matrix (and the ±λ proof)
:icon: unfold

Pontryagin's principle gives a costate $\boldsymbol\lambda = P\mathbf x$ and the
joint linear system

$$
\begin{bmatrix}\dot{\mathbf x}\\ \dot{\boldsymbol\lambda}\end{bmatrix}
= \underbrace{\begin{bmatrix} A & -BR^{-1}B^\top\\ -Q & -A^\top \end{bmatrix}}_{\mathcal H}
\begin{bmatrix}\mathbf x\\ \boldsymbol\lambda\end{bmatrix}.
$$

**Claim.** The eigenvalues of $\mathcal H$ come in pairs $\pm\lambda$.

*Proof.* With $\mathcal J = \begin{bmatrix}0 & I\\ -I & 0\end{bmatrix}$, the matrix
$\mathcal J\mathcal H = \begin{bmatrix}-Q & -A^\top\\ -A & BR^{-1}B^\top\end{bmatrix}$
is symmetric. Hence $\mathcal H^\top = -\mathcal J\mathcal H\mathcal J^{-1}$, so
$\mathcal H$ is similar to $-\mathcal H^\top$, which has the eigenvalues of
$\mathcal H$ negated. $\square$

So exactly half of the $2n$ eigenvalues are stable. The optimal trajectory
must decay, so it lives in the **stable invariant subspace**. If the columns of
$\begin{bmatrix}X_1\\ X_2\end{bmatrix}$ span that subspace, every solution in it
satisfies $\boldsymbol\lambda = X_2 X_1^{-1}\mathbf x$. Comparing with
$\boldsymbol\lambda = P\mathbf x$:

$$
P = X_2 X_1^{-1}.
$$

{func}`~drone_sim.control.lqr.solve_care` does exactly this: it
eigendecomposes $\mathcal H$, takes the $n$ eigenvectors with the most
negative real parts, forms $X_2X_1^{-1}$, and symmetrizes away rounding error.
It raises an error if fewer than $n$ eigenvalues are strictly stable. The
doctest checks the scalar case $\dot x = u$, $Q = R = 1$, where the CARE
$-P^2 + 1 = 0$ gives $P = 1$.
::::

::::{dropdown} Step 4: using the gain
:icon: unfold

$$
\mathbf u = \mathbf u_{eq} - K(\mathbf x - \mathbf x_{\text{ref}}),
$$

with the yaw error wrapped into $(-\pi, \pi]$ and thrust clipped at zero
({meth}`~drone_sim.control.lqr.LQRController.control`).
::::

:::{admonition} Why it matters for the drone
:class: aerl-why
- In the recovery demo the drone starts 5 m away and pitches to about **58°**,
  far from the hover the LQR was designed for. It still converges, because
  LQR designs have generous stability margins (for full-state feedback, at
  least 60° of phase margin per input channel). It is no longer optimal
  there, though, and motor saturation clips its commands ({doc}`/demos/other`).
- On the moving figure-eight it tracks 2–10× worse than the cascaded PID: it
  was designed to hold a point, not to follow a path (finding 11 on the
  {doc}`/findings/index` page).
:::
