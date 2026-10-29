# Quaternions

:::{admonition} In plain words
:class: aerl-plain
A quaternion describes an orientation with **four numbers** instead of three
angles. Think of it as "turn by angle α around axis **n**", packed into one
object. It never suffers gimbal lock, two rotations combine by a single
multiplication, and updating it from a gyroscope is a short, well-behaved
formula. Both of this project's estimators store the drone's attitude as a
quaternion. Euler angles are only computed for display.
:::

```{localvideo} theory/v1_rotations.mp4
:title: Euler angles, gimbal lock, and quaternions
```

## The key idea

A rotation by angle $\alpha$ about the unit axis $\mathbf n$ is the unit
quaternion

$$
q = \big(\cos\tfrac{\alpha}{2},\; \sin\tfrac{\alpha}{2}\,\mathbf n\big),
\qquad \|q\| = 1 .
$$

It rotates a vector $\mathbf u$ by "sandwiching" it, and the gyroscope's spin
rate $\boldsymbol\omega$ updates it over time:

$$
(0, \mathbf u') = q \otimes (0, \mathbf u) \otimes q^*,
\qquad
\dot q = \tfrac12\, q \otimes (0, \boldsymbol\omega).
$$

| Symbol | Meaning |
|---|---|
| $q = (w, x, y, z)$ | the quaternion: one scalar part $w$ and a vector part $(x, y, z)$ |
| $\mathbf n$, $\alpha$ | rotation axis (unit vector) and angle |
| $\otimes$ | quaternion multiplication (order matters, like rotations) |
| $q^*$ | conjugate $(w, -x, -y, -z)$, the reverse rotation |
| $\boldsymbol\omega$ | body spin rate from the gyro (rad/s) |
| $\dot q$ | how fast the quaternion changes |

There is no $\tan$ and no division by $\cos$ anywhere, so the update works for
**every** attitude, including pointing straight up. A small catch: one
discrete update step makes $q$ slightly too long, so the code divides it by its
length after every step.

In code: {class}`~drone_sim.dynamics.quaternions.Quaternion`, stored scalar
first, `[w, x, y, z]`.

## Show the derivations

::::{dropdown} The algebra: the Hamilton product
:icon: unfold

A quaternion is $q = w + x\,i + y\,j + z\,k$ with
$i^2 = j^2 = k^2 = ijk = -1$. Writing $q = (w, \mathbf v)$ with scalar $w$ and
vector $\mathbf v = [x, y, z]$, the **Hamilton product** is

$$
(w_1, \mathbf v_1) \otimes (w_2, \mathbf v_2) =
\big(w_1 w_2 - \mathbf v_1\!\cdot\!\mathbf v_2,\;
w_1 \mathbf v_2 + w_2 \mathbf v_1 + \mathbf v_1 \times \mathbf v_2\big).
$$

The cross product makes it **non-commutative**: $i \otimes j = k$ but
$j \otimes i = -k$. The conjugate is $q^* = (w, -\mathbf v)$, the norm is
$\|q\| = \sqrt{q \otimes q^*}$, and norms multiply:
$\|p \otimes q\| = \|p\|\,\|q\|$.
::::

::::{dropdown} Proof: the sandwich product is a rotation (Rodrigues' formula)
:icon: unfold

**Claim.** $\mathbf u'$ is $\mathbf u$ rotated by $\alpha$ about $\mathbf n$.

*Proof.* Expanding the products with $q = (w, \mathbf v)$, $\|q\| = 1$:

$$
\mathbf u' = (w^2 - \mathbf v\!\cdot\!\mathbf v)\,\mathbf u
+ 2(\mathbf v\!\cdot\!\mathbf u)\,\mathbf v + 2w\,(\mathbf v \times \mathbf u).
$$

Substitute $w = \cos\frac\alpha2$, $\mathbf v = \sin\frac\alpha2\,\mathbf n$ and
the half-angle identities $\cos^2 - \sin^2 = \cos\alpha$,
$2\sin^2 = 1 - \cos\alpha$, $2\sin\cos = \sin\alpha$:

$$
\mathbf u' = \cos\alpha\,\mathbf u + (1-\cos\alpha)(\mathbf n\!\cdot\!\mathbf u)\,\mathbf n
+ \sin\alpha\,(\mathbf n \times \mathbf u),
$$

which is Rodrigues' rotation formula. $\square$

The EKF uses an equivalent form that avoids two full products,
$\mathbf u' = \mathbf u + 2\mathbf v \times (\mathbf v \times \mathbf u + w\,\mathbf u)$
({func}`~drone_sim.estimation.ekf.rotate_vector`).

Two consequences:

- **Composition** is multiplication: rotating by $q_1$ and then by $q_2$ is
  the single quaternion $q_2 \otimes q_1$.
- **Double cover**: $q$ and $-q$ give the same rotation, since the sandwich
  product is quadratic in $q$. The filters never compare quaternions
  directly; they compare predicted measurements, which are sign-invariant.
::::

::::{dropdown} Derivation: integrating the gyro, and why q must be renormalized
:icon: unfold

Let $q(t)$ map body to world, and let the gyro measure $\boldsymbol\omega$ in
body axes. During $\delta t$ the body turns by the small body-frame rotation
$\delta q \approx (1, \tfrac12 \boldsymbol\omega\,\delta t)$. A body-frame rotation
composes on the right:

$$
q(t+\delta t) = q(t) \otimes \delta q
\;\Rightarrow\;
\dot q = \tfrac12\, q \otimes (0, \boldsymbol\omega).
$$

This is the equation behind
{meth}`MadgwickMARG.integrate_gyro_only <drone_sim.estimation.madgwick.MadgwickMARG.integrate_gyro_only>`
and {meth}`IMUEKF.transition <drone_sim.estimation.ekf.IMUEKF.transition>`.

**Claim.** The exact flow preserves $\|q\|$.

*Proof.* $\frac{d}{dt}\|q\|^2 = 2\langle q, \dot q\rangle
= \langle q, q \otimes (0, \boldsymbol\omega)\rangle$. Left multiplication by
$q$ scales inner products by $\|q\|^2$, so this equals
$\|q\|^2\,\langle (1, \mathbf 0), (0, \boldsymbol\omega)\rangle = 0$. $\square$

**But the discrete step does not.** One explicit Euler step gives
$q + \tfrac12 q\otimes(0,\boldsymbol\omega)\,\delta t = q \otimes (1, \tfrac12\boldsymbol\omega\,\delta t)$,
whose norm is $\|q\|\sqrt{1 + \|\boldsymbol\omega\|^2 \delta t^2/4} > \|q\|$.
The error is second order but it accumulates, so every update in this project
renormalizes $q$. The EKF also projects its covariance through the
normalization ({doc}`ekf`).
::::

::::{dropdown} Conversions to rotation matrices and Euler angles
:icon: unfold

To a rotation matrix (used by the live viewer to draw the board):

$$
R(q) = \begin{bmatrix}
1-2(y^2+z^2) & 2(xy - wz) & 2(xz + wy)\\
2(xy + wz) & 1-2(x^2+z^2) & 2(yz - wx)\\
2(xz - wy) & 2(yz + wx) & 1-2(x^2+y^2)
\end{bmatrix}.
$$

To and from ZYX Euler angles:
{func}`~drone_sim.dynamics.plant.euler_to_quaternion` and
{func}`~drone_sim.dynamics.plant.quaternion_to_euler`. The reverse direction
uses $\theta = \arcsin\big(2(wy - zx)\big)$, which is exactly where the
Euler singularity reappears. The project converts to Euler angles only for
display and for comparison with the plant.
::::

::::{dropdown} Why not just use rotation matrices?
:icon: unfold

A rotation matrix is also singularity-free, but it has 9 numbers and 6
constraints. Keeping it orthonormal after integration takes a costly
re-orthogonalization. A quaternion has 4 numbers and one constraint
(renormalize by dividing by the norm), and it is cheap to differentiate. That
matters inside the EKF's Jacobians.
::::

:::{admonition} Why it matters for the drone
:class: aerl-why
- The live viewer keeps working when the board points straight up, because its
  EKF integrates a quaternion, not Euler angles.
- The "one step makes q too long" detail is real: it is part of the error
  traced in finding 4 on the {doc}`/findings/index` page, and it is why every
  filter update in the code ends by renormalizing $q$.
:::
