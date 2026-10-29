# Coordinate frames and specific force

:::{admonition} In plain words
:class: aerl-plain
Every vector needs a reference frame: which way is "x", and does "z" point up
or down? The physics model uses the aerospace convention (**z points down**),
while the sensor and filters use **z up**. The project converts between them in
exactly one place. This page also explains a surprise: an accelerometer does
**not** measure gravity. It measures everything *except* gravity, which is why
it can't tell a tilted drone from an accelerating one.
:::

## The key idea

| | World frame | Body frame | Used by |
|---|---|---|---|
| **Plant** | NED: x North, y East, z **Down** | FRD: x Forward, y Right, z **Down** | {mod}`~drone_sim.dynamics`, {mod}`~drone_sim.control` |
| **Filters** | x, y, z **Up** | FLU: x Forward, y Left, z **Up** | {mod}`~drone_sim.estimation`, IMU stream |

The two conventions differ by a half turn about the x axis:

$$
S = \operatorname{diag}(1, -1, -1), \qquad v_{\text{filter}} = S\,v_{\text{plant}}.
$$

What an accelerometer reads is the **specific force**: the acceleration
minus gravity, seen from the body:

$$
\mathbf f_{\text{body}} = R^\top \left(\mathbf a_{\text{world}} - \mathbf g_{\text{world}}\right).
$$

| Symbol | Meaning |
|---|---|
| $S$ | frame-change matrix (a 180° turn about x) |
| $R$ | body-to-world rotation; $R^\top$ goes from world to body |
| $\mathbf a_{\text{world}}$ | the body's actual acceleration (m/s²) |
| $\mathbf g_{\text{world}}$ | gravity, $[0, 0, g]$ in NED (m/s²) |
| $\mathbf f_{\text{body}}$ | what the accelerometer reports (m/s²) |

Some worked examples:

| Situation | $\mathbf a_{\text{world}}$ | Accelerometer reads (FLU) |
|---|---|---|
| Resting on a table | $0$ | $[0, 0, +g]$: the table's push |
| Free fall | $\mathbf g$ | $[0, 0, 0]$: nothing pushes |
| Hovering drone | $0$ | $[0, 0, +g]$: thrust replaces the table |
| Accelerating forward at $a$, level | $[a, 0, 0]$ | $[a, 0, +g]$: **looks tilted** |

The last row is the central problem of IMU-only attitude estimation: during
acceleration the reading tilts by $\arctan(a/g)$ even though the drone is
level.

## Show the derivations

::::{dropdown} Why S is a safe conversion (a proper rotation)
:icon: unfold

**Claim.** $S$ is a proper rotation, so it preserves handedness and cross
products.

*Proof.* $S^\top S = I$ and $\det S = (1)(-1)(-1) = +1$. In fact
$S = R_x(\pi)$, a half turn about the shared x axis. $\square$

Vectors therefore transform as $v' = S v$, and $S^{-1} = S$, so every
conversion function is its own inverse.

**Quaternions.** A rotation $R$ expressed in the new frames is
$R' = S R S^\top$. For $q = [w, x, y, z]$, conjugating by a half turn about x
leaves the x component alone and negates y and z:

$$
q' = [\,w,\; x,\; -y,\; -z\,].
$$

This is {func}`~drone_sim.dynamics.frames.ned_frd_quaternion_to_up_flu`. The
test `test_rotation_and_vector_equivalence_under_frame_change` checks that
rotating a converted vector with the converted quaternion gives the same
answer as converting the rotated vector.
::::

::::{dropdown} The body-to-world rotation and Newton's law in NED
:icon: unfold

With ZYX Euler angles (yaw, then pitch, then roll):

$$
R = R_z(\psi)\, R_y(\theta)\, R_x(\phi),
$$

where each factor is a right-handed elementary rotation
({func}`~drone_sim.dynamics.plant.rotation_x` etc.). $R$ maps body
coordinates to world coordinates: $v_{\text{world}} = R\, v_{\text{body}}$. The
inverse is the transpose, $R^{-1} = R^\top$, *not* the product of the factors
in reverse order with the same signs.

Newton's second law in the NED world frame, with thrust along body $-z$ and
linear drag $k_d$:

$$
\dot{\mathbf v} = \frac{1}{m}\Big( R \begin{bmatrix}0\\0\\-T\end{bmatrix} - k_d \mathbf v \Big)
+ \begin{bmatrix}0\\0\\g\end{bmatrix}.
$$

At level hover ($R = I$, $\mathbf v = 0$), $T = mg$ gives
$\dot{\mathbf v} = 0$, as it should.
::::

::::{dropdown} Why an accelerometer cannot feel gravity
:icon: unfold

An accelerometer is a small proof mass on a spring inside a case. Gravity pulls
the mass and the case equally, so it does not stretch the spring. Only the
*other* forces on the case (a table, thrust, drag) do. The reading is
therefore the total acceleration minus gravity, expressed in the sensor's own
axes: $\mathbf f_{\text{body}} = R^\top(\mathbf a_{\text{world}} - \mathbf g_{\text{world}})$.

The pipeline computes the true specific force exactly this way
({func}`~drone_sim.simulation.pipeline.run_pipeline`): it takes the plant's
acceleration from the state derivative, subtracts gravity, rotates into FRD,
and converts to FLU with $S$.
::::

:::{admonition} Why it matters for the drone
:class: aerl-why
- A sign error between the two frames once made the plant and the EKF disagree
  about which way gravity points. Keeping every conversion in
  {mod}`drone_sim.dynamics.frames` fixed it (see {doc}`/findings/index`).
- Because a tilted drone and an accelerating drone read the same, the filters
  take a `gravity_valid` flag and ignore the accelerometer's "down" during
  powered flight.
:::
