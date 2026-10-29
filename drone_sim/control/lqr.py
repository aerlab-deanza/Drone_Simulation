"""Linear-quadratic regulator (LQR) linearized about hover.

Steps:

1. Pick the hover equilibrium: level attitude, zero velocity, thrust = weight.
2. Linearize the nonlinear dynamics there with central finite differences,
   giving ``δẋ = A δx + B δu``.
3. Solve the continuous-time algebraic Riccati equation (CARE)::

       Aᵀ P + P A - P B R⁻¹ Bᵀ P + Q = 0

   directly, using the stable invariant subspace of the Hamiltonian matrix
   (no ``scipy.linalg.solve_continuous_are``).
4. Form the optimal gain ``K = R⁻¹ Bᵀ P`` and apply ``u = u_eq - K (x - x_ref)``.

State (same as :mod:`drone_sim.dynamics.plant`, NED/FRD)::

    x = [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]

Control (virtual actuators)::

    u = [thrust, tau_roll, tau_pitch, tau_yaw]

Run ``python -m drone_sim.control.lqr`` for a 20-second recovery demo that saves
``outputs/lqr_response.png``.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from ..dynamics.plant import (
    GRAVITY_MPS2,
    DroneConfig,
    DronePlant,
    body_to_world_rotation,
    euler_rate_matrix,
)

#: Number of states in the plant model.
N_STATES = 12
#: Number of virtual control inputs.
N_INPUTS = 4


def nonlinear_dynamics(state: ArrayLike, control: ArrayLike,
                       config: DroneConfig) -> np.ndarray:
    """Return ``dx/dt`` of the plant model as a pure function.

    This matches :meth:`DronePlant.state_derivative
    <drone_sim.dynamics.plant.DronePlant.state_derivative>`, written without a
    plant instance so it can be differentiated numerically.

    Args:
        state: 12-element state vector (NED/FRD).
        control: ``[thrust (N), τx, τy, τz (N·m)]``.
        config: Drone parameters.

    Returns:
        12-element state derivative.
    """
    _, _, _, vx, vy, vz, phi, theta, psi, wx, wy, wz = state
    thrust, tau_x, tau_y, tau_z = control

    thrust_world = body_to_world_rotation(phi, theta, psi) @ np.array([0.0, 0.0, -thrust])
    velocity = np.array([vx, vy, vz])
    acceleration = ((thrust_world - config.drag_coefficient * velocity) / config.mass
                    + np.array([0.0, 0.0, GRAVITY_MPS2]))

    euler_rates = euler_rate_matrix(phi, theta) @ np.array([wx, wy, wz])

    ixx, iyy, izz = np.diag(config.inertia)
    angular_acceleration = np.array([
        ((iyy - izz) * wy * wz + tau_x) / ixx,
        ((izz - ixx) * wx * wz + tau_y) / iyy,
        ((ixx - iyy) * wx * wy + tau_z) / izz,
    ])

    return np.concatenate([velocity, acceleration, euler_rates, angular_acceleration])


def hover_equilibrium(config: DroneConfig) -> tuple[np.ndarray, np.ndarray]:
    """Return the hover equilibrium ``(x_eq, u_eq)``.

    Args:
        config: Drone parameters.

    Returns:
        Zero state, and ``u_eq = [m·g, 0, 0, 0]``.
    """
    equilibrium_state = np.zeros(N_STATES)
    equilibrium_control = np.array([config.mass * GRAVITY_MPS2, 0.0, 0.0, 0.0])
    return equilibrium_state, equilibrium_control


def linearize(config: DroneConfig, equilibrium_state: np.ndarray,
              equilibrium_control: np.ndarray,
              epsilon: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    """Return the Jacobians ``A = ∂f/∂x`` and ``B = ∂f/∂u`` at an equilibrium.

    Uses central differences ``(f(x+ε) - f(x-ε)) / 2ε`` (error ``O(ε²)``).

    Args:
        config: Drone parameters.
        equilibrium_state: Linearization state (12,).
        equilibrium_control: Linearization input (4,).
        epsilon: Finite-difference step.

    Returns:
        ``(A, B)`` with shapes ``(12, 12)`` and ``(12, 4)``.
    """
    a_matrix = np.zeros((N_STATES, N_STATES))
    b_matrix = np.zeros((N_STATES, N_INPUTS))

    for i in range(N_STATES):
        step = np.zeros(N_STATES)
        step[i] = epsilon
        f_plus = nonlinear_dynamics(equilibrium_state + step, equilibrium_control, config)
        f_minus = nonlinear_dynamics(equilibrium_state - step, equilibrium_control, config)
        a_matrix[:, i] = (f_plus - f_minus) / (2 * epsilon)

    for i in range(N_INPUTS):
        step = np.zeros(N_INPUTS)
        step[i] = epsilon
        f_plus = nonlinear_dynamics(equilibrium_state, equilibrium_control + step, config)
        f_minus = nonlinear_dynamics(equilibrium_state, equilibrium_control - step, config)
        b_matrix[:, i] = (f_plus - f_minus) / (2 * epsilon)

    return a_matrix, b_matrix


def solve_care(a_matrix: np.ndarray, b_matrix: np.ndarray,
               q_matrix: np.ndarray, r_matrix: np.ndarray) -> np.ndarray:
    """Solve the continuous algebraic Riccati equation via the Hamiltonian.

    The Hamiltonian ``H = [[A, -B R⁻¹ Bᵀ], [-Q, -Aᵀ]]`` has eigenvalues in
    ``±λ`` pairs. If the columns ``[X1; X2]`` span its stable invariant
    subspace, then ``P = X2 X1⁻¹`` is the stabilizing solution.

    Args:
        a_matrix: State matrix ``A`` (n×n).
        b_matrix: Input matrix ``B`` (n×m).
        q_matrix: State weight ``Q`` (n×n, positive semidefinite).
        r_matrix: Input weight ``R`` (m×m, positive definite).

    Returns:
        Symmetric solution ``P`` (n×n).

    Raises:
        ValueError: If ``H`` lacks n strictly stable eigenvalues, meaning
            ``(A, B)`` is not stabilizable or ``(A, Q^½)`` not detectable.

    Example:
        >>> # Scalar plant ẋ = u with Q = R = 1 has P = 1.
        >>> solve_care(np.array([[0.0]]), np.array([[1.0]]),
        ...            np.array([[1.0]]), np.array([[1.0]])).round(9).tolist()
        [[1.0]]
    """
    n = a_matrix.shape[0]
    r_inverse = np.linalg.inv(r_matrix)

    hamiltonian = np.block([
        [a_matrix, -b_matrix @ r_inverse @ b_matrix.T],
        [-q_matrix, -a_matrix.T],
    ])
    eigenvalues, eigenvectors = np.linalg.eig(hamiltonian)

    # Indices of the n eigenvalues with the most negative real part.
    stable_indices = np.argsort(eigenvalues.real)[:n]
    if np.any(eigenvalues[stable_indices].real >= -1e-9):
        raise ValueError(
            "Hamiltonian doesn't have n eigenvalues with negative real part "
            "-- check that (A, B) is stabilizable and (A, Q^{1/2}) is detectable."
        )

    stable_subspace = eigenvectors[:, stable_indices]
    x1 = stable_subspace[:n, :]
    x2 = stable_subspace[n:, :]
    solution = x2 @ np.linalg.inv(x1)
    solution = (solution + solution.conj().T) / 2
    return np.real(solution)


class LQRController:
    """LQR full-state feedback about hover.

    Args:
        config: Drone parameters.
        q_matrix: 12×12 state weight. Defaults to
            ``diag(10, 10, 10, 1, 1, 1, 50, 50, 20, 1, 1, 1)``.
        r_matrix: 4×4 input weight. Defaults to ``diag(0.5, 20, 20, 20)``.

    Attributes:
        a_matrix: Linearized state matrix ``A``.
        b_matrix: Linearized input matrix ``B``.
        riccati_solution: CARE solution ``P``.
        gain: Optimal gain ``K = R⁻¹ Bᵀ P`` (4×12).

    Example:
        >>> config = DroneConfig(1.2, np.diag([0.02, 0.02, 0.04]), 0.25, 0.01, 3e-5, 1e-6)
        >>> controller = LQRController(config)
        >>> bool(np.all(controller.closed_loop_eigenvalues().real < 0))
        True
    """

    def __init__(self, config: DroneConfig, q_matrix: np.ndarray | None = None,
                 r_matrix: np.ndarray | None = None) -> None:
        self.config = config

        self.equilibrium_state, self.equilibrium_control = hover_equilibrium(config)
        self.a_matrix, self.b_matrix = linearize(
            config, self.equilibrium_state, self.equilibrium_control)

        if q_matrix is None:
            # state order: [x,y,z, vx,vy,vz, phi,theta,psi, wx,wy,wz]
            q_matrix = np.diag([10, 10, 10, 1, 1, 1, 50, 50, 20, 1, 1, 1]).astype(float)
        if r_matrix is None:
            # input order: [thrust, tau_roll, tau_pitch, tau_yaw]
            r_matrix = np.diag([0.5, 20, 20, 20]).astype(float)
        self.q_matrix = q_matrix
        self.r_matrix = r_matrix

        self.riccati_solution = solve_care(self.a_matrix, self.b_matrix,
                                           self.q_matrix, self.r_matrix)
        self.gain = np.linalg.inv(self.r_matrix) @ self.b_matrix.T @ self.riccati_solution

        # Used only for its motor-allocation helper.
        self._mixer = DronePlant(config, np.zeros(N_STATES))

    def closed_loop_eigenvalues(self) -> np.ndarray:
        """Return the eigenvalues of ``A - B K``; all have negative real part."""
        return np.linalg.eigvals(self.a_matrix - self.b_matrix @ self.gain)

    def control(self, state: ArrayLike, reference: ArrayLike | None = None) -> np.ndarray:
        """Return the LQR command for the current state.

        Args:
            state: 12-element plant state (NED/FRD).
            reference: 12-element target state. Defaults to hover at the
                origin.

        Returns:
            ``[thrust (N), τ_roll, τ_pitch, τ_yaw (N·m)]``. Thrust is clipped at
            zero; yaw error is wrapped into ``(-π, π]``.
        """
        if reference is None:
            reference = self.equilibrium_state

        error = np.asarray(state, dtype=float) - np.asarray(reference, dtype=float)
        # Wrap the yaw error so the controller never turns the long way round.
        error[8] = np.arctan2(np.sin(error[8]), np.cos(error[8]))

        command = self.equilibrium_control - self.gain @ error
        command[0] = max(command[0], 0.0)
        return command

    def control_motor_speeds(self, state: ArrayLike,
                             reference: ArrayLike | None = None) -> np.ndarray:
        """Return :meth:`control` converted to four motor speeds (rad/s)."""
        command = self.control(state, reference)
        return self._mixer.allocate_control(command[0], command[1:4])[2]


def run_recovery_demo(output_path: str = "outputs/lqr_response.png") -> None:
    """Simulate LQR recovery from a tilted, offset, spinning start and plot it.

    Args:
        output_path: Where to save the 2×2 response figure (PNG).
    """
    import matplotlib.pyplot as plt

    config = DroneConfig(mass=1.2, inertia=np.diag([0.02, 0.02, 0.04]),
                         arm_length=0.25, drag_coefficient=0.01,
                         thrust_coefficient=3e-5, yaw_torque_coefficient=1e-6)
    controller = LQRController(config)

    print("Closed-loop eigenvalues (should all have negative real part):")
    print(np.round(controller.closed_loop_eigenvalues(), 3))

    initial_state = np.zeros(N_STATES)
    initial_state[0:3] = [5.0, -0.5, 2.0]     # position offset (m)
    initial_state[6:9] = [0.3, -0.2, 0.5]     # roll, pitch, yaw offset (rad)
    initial_state[9:12] = [0.1, -0.1, 0.05]   # body rates (rad/s)
    plant = DronePlant(config, initial_state.copy())

    dt, steps = 0.01, 2000  # 20 seconds
    history = np.zeros((steps + 1, N_STATES))
    history[0] = initial_state
    state = initial_state.copy()
    for k in range(steps):
        command = controller.control(state)
        thrust, torques, _ = plant.allocate_control(command[0], command[1:4])
        state = plant.step(thrust, torques, dt)
        history[k + 1] = state

    print("\nFinal state [x,y,z, vx,vy,vz, phi,theta,psi, wx,wy,wz]:")
    print(np.round(history[-1], 4))

    t = np.arange(steps + 1) * dt
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    panels = [((0, 0), slice(0, 3), "Position (m)", ["x", "y", "z"]),
              ((0, 1), slice(6, 9), "Attitude (rad)", ["phi", "theta", "psi"]),
              ((1, 0), slice(3, 6), "Velocity (m/s)", ["vx", "vy", "vz"]),
              ((1, 1), slice(9, 12), "Body rates (rad/s)", ["wx", "wy", "wz"])]
    for (row, col), columns, title, legend in panels:
        axes[row, col].plot(t, history[:, columns])
        axes[row, col].set_title(title)
        axes[row, col].legend(legend)
    for ax in axes.flat:
        ax.set_xlabel("time (s)")
        ax.grid(True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    print(f"\nSaved response plot to {output_path}")


if __name__ == "__main__":
    run_recovery_demo()
