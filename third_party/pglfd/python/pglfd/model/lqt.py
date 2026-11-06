# Copyright (c) 2026 Maximilian Mühlbauer
#
# This file is part of pglfd.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""LQT on manifold implementation :cite:p:`muehlbauer2026geometric`, :cite:p:`muehlbauer2026lqt`."""

from typing import Any

import numpy as np

from pglfd.manif.manifold import Manifold
from pglfd.model.gaussian import Gaussian

try:
    from pglfd_cpp.lqt_wrapper import get_cpp_lqt

    have_cpp = True
except ImportError:
    have_cpp = False


class LQT:
    """Implementation of LQT on a manifold."""

    def __new__(
        cls,
        manifold: Manifold,
        initial_state: np.ndarray,
        t_vias: np.ndarray,
        vias: list[Gaussian],
        tend: float = 1.0,
        dt: float = 0.01,
        rfactor: float | np.ndarray = 1e-4,
        initial_velocity: np.ndarray | None = None,
    ) -> Any:
        """
        Initialize the LQT.

        :param manifold: The manifold to work on.
        :param initial_state: The initial state.
        :param t_vias: Timepoints of vias.
        :param vias: The vias.
        :param tend: End time of LQT.
        :param dt: Time between two steps.
        :param rfactor: R factor.
        :param initial_velocity: Initial velocity.
        """
        # Check inputs
        if t_vias.shape[0] != len(vias):
            msg = f"Got {t_vias.shape[0]} timestamps for {len(vias)} via points!"
            raise ValueError(msg)

        if have_cpp:
            lqt = get_cpp_lqt(manifold, initial_state, t_vias, vias, tend, dt, rfactor, initial_velocity)
            if lqt is not None:
                return lqt
        return object.__new__(cls)

    def __getnewargs__(self) -> tuple:
        """
        Get args for pickling.

        :return: manifold, initial_state, t_vias, vias, tend, dt, rfactor, initial_velocity
        """
        return (
            self.manifold,
            self.initial_state,
            self.t_vias,
            self.vias,
            self.tend,
            self.dt,
            self.rfactor,
            self.initial_velocity,
        )

    def __init__(
        self,
        manifold: Manifold,
        initial_state: np.ndarray,
        t_vias: np.ndarray,
        vias: list[Gaussian],
        tend: float = 1.0,
        dt: float = 0.01,
        rfactor: float | np.ndarray = 1e-4,
        initial_velocity: np.ndarray | None = None,
    ) -> None:
        """
        Initialize the LQT.

        :param manifold: The manifold to work on.
        :param initial_state: The initial state.
        :param t_vias: Timepoints of vias.
        :param vias: The vias.
        :param tend: End time of LQT.
        :param dt: Time between two steps.
        :param rfactor: R factor.
        :param initial_velocity: Initial velocity.
        """
        #: The manifold of the LQT.
        self.manifold = manifold
        #: Time between steps.
        self.dt = dt
        self.tend = tend
        self.ts = np.linspace(0, self.tend, int(self.tend / self.dt) + 1)

        self.initial_state = initial_state
        if initial_velocity is not None:
            self.initial_velocity = initial_velocity
        else:
            self.initial_velocity = np.zeros(self.manifold.dim_tangent)
        self.rfactor = np.diag(np.ones(self.manifold.dim_tangent) * rfactor)

        # check vias
        self.t_vias = t_vias
        self.idx_vias = []
        self.vias = vias
        for i in range(len(self.t_vias)):
            closest = np.argmin(np.abs(self.ts - self.t_vias[i]))
            self.idx_vias.append(closest)
            self.t_vias[i] = self.ts[closest]

        #: Control commands at each time step.
        self.commands = np.zeros((self.ts.shape[0] - 1, self.manifold.dim_tangent))
        #: Velocities at each time step.
        self.velocities = np.zeros((self.ts.shape[0], self.manifold.dim_tangent))
        #: Poses at each time step.
        self.poses = np.zeros((self.ts.shape[0], self.manifold.dim_manifold))
        #: States at each time step.
        self.states = [
            Gaussian(self.manifold, self.manifold.id_elem, np.eye(self.manifold.dim_tangent))
            for _ in range(self.ts.shape[0] - 1)
        ]

        # allocate su, precompute q
        self.su_reduced = np.zeros((len(vias) * self.manifold.dim_tangent, self.commands.shape[0] * self.manifold.dim_tangent))
        self.su = np.zeros((self.ts.shape[0] * self.manifold.dim_tangent, self.commands.shape[0] * self.manifold.dim_tangent))
        self.q = np.zeros((len(vias) * self.manifold.dim_tangent, len(vias) * self.manifold.dim_tangent))
        for idx_via, via in zip(range(len(vias)), vias, strict=True):
            idx_start = idx_via * self.manifold.dim_tangent
            idx_end = idx_start + self.manifold.dim_tangent
            self.q[idx_start:idx_end, idx_start:idx_end] = np.linalg.inv(via.sigma)

        # precompute r
        self.r = np.kron(np.eye(self.commands.shape[0]), self.rfactor)

        #: Indicates whether this LQT is using a C++ implementation.
        self.is_cpp: bool = False

    def _calc_su(self, su: np.ndarray, ivals: range, idx_states: range | list) -> None:
        for i, idx_state in zip(ivals, idx_states, strict=True):
            idx_start = i * self.manifold.dim_tangent
            idx_end = idx_start + self.manifold.dim_tangent
            for j in range(idx_state - 1):
                jdx_start = j * self.manifold.dim_tangent
                jdx_end = jdx_start + self.manifold.dim_tangent
                trafo = np.eye(self.manifold.dim_tangent)
                for k in range(j + 1, idx_state + 1):
                    trafo = self.manifold.parallel_transport_trafo(self.poses[k - 1], self.poses[k]) @ trafo
                su[idx_start:idx_end, jdx_start:jdx_end] = trafo * (self.ts[idx_state] - self.ts[j] - self.dt) * self.dt

    def _calc_covariance(self) -> None:
        # full su
        self._calc_su(self.su, range(1, self.ts.shape[0]), range(1, self.ts.shape[0]))

        # covariance
        cov = self.su @ np.linalg.inv(self.su_reduced.T @ self.q @ self.su_reduced + self.r) @ self.su.T
        for i in range(1, self.ts.shape[0]):
            idx_start = i * self.manifold.dim_tangent
            idx_end = idx_start + self.manifold.dim_tangent
            self.states[i - 1] = Gaussian(self.manifold, self.poses[i], cov[idx_start:idx_end, idx_start:idx_end])

    def _rollout(self, poses: np.ndarray, velocities: np.ndarray, commands: np.ndarray) -> None:
        poses[0] = self.initial_state
        velocities[0] = self.initial_velocity
        for i in range(1, self.ts.shape[0]):
            # next pose
            poses[i] = self.manifold.exp(velocities[i - 1] * self.dt, poses[i - 1])
            # next velocity
            velocities[i] = self.manifold.parallel_transport_vector(
                velocities[i - 1] + commands[i - 1] * self.dt, poses[i - 1], poses[i]
            )

    def _calc_cost(self, commands: np.ndarray) -> tuple:
        # initial rollout
        poses = np.zeros_like(self.poses)
        velocities = np.zeros_like(self.velocities)
        self._rollout(poses, velocities, commands)

        # calculate via point cost
        cost = np.zeros(len(self.vias))
        for i, idx_state in zip(range(len(self.vias)), self.idx_vias, strict=True):
            logval = self.manifold.log(poses[idx_state], self.vias[i].mu)
            cost[i] = logval.T @ np.linalg.inv(self.vias[i].sigma) @ logval

        # calculate total cost
        total_cost = cost.sum() + commands.reshape(-1).T @ self.r @ commands.reshape(-1)

        # calculate total len
        total_len = 0.0
        for i in range(self.poses.shape[0] - 1):
            total_len += float(np.linalg.norm(self.velocities[i, :] * self.dt))

        return total_cost, total_len

    def _calc_commands(self) -> float:
        # reduced su
        self._calc_su(self.su_reduced, range(len(self.vias)), self.idx_vias)

        # preallocate matrices
        logs = np.zeros(len(self.vias) * self.manifold.dim_tangent)

        # fill logs
        for idx_via, idx_state, via in zip(range(len(self.vias)), self.idx_vias, self.vias, strict=True):
            idx_start = idx_via * self.manifold.dim_tangent
            idx_end = idx_start + self.manifold.dim_tangent
            logs[idx_start:idx_end] = self.manifold.log(self.poses[idx_state], via.mu)

        # calculate eps
        eps = -(
            np.linalg.inv(self.su_reduced.T @ self.q @ self.su_reduced + self.r)
            @ (self.su_reduced.T @ self.q @ logs + self.r @ self.commands.reshape(-1))
        ).reshape((self.commands.shape[0], self.manifold.dim_tangent))

        # line search
        last_cost, last_len = self._calc_cost(self.commands)
        cur_cost, cur_len = self._calc_cost(self.commands + eps)
        scaler = 1.0
        while cur_cost > last_cost and cur_len > 2.0 * last_len:
            scaler /= 2.0
            cur_cost, cur_len = self._calc_cost(self.commands + scaler * eps)

        # update & return
        self.commands += scaler * eps
        return np.linalg.norm(eps) / self.commands.shape[0]  # type: ignore[return-value]

    def fit(
        self,
        eps: float = 1e-8,
        maxit: int = 50,
    ) -> int:
        """
        Fit the LQT.

        :param eps: Stopping criterion.
        :param maxit: Maximum number of iterations to perform before exiting.
        :return: Steps required for fitting.
        """
        # Iterate rollout - update commands
        self._rollout(self.poses, self.velocities, self.commands)
        it = 0
        while self._calc_commands() > eps:
            self._rollout(self.poses, self.velocities, self.commands)

            it += 1
            if it > maxit:
                msg = f"LQT fitting did not converge in {maxit} steps"
                raise RuntimeWarning(msg)

        # Finally, calculate the covariance
        self._calc_covariance()

        return it
