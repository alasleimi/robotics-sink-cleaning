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

"""
KMP on Riemannian manifolds.

:cite:t:`muehlbauer2026nonparametric`
:cite:t:`muehlbauer2026geometric`.
"""

from typing import Any

import numpy as np

from pglfd.manif.manifold import Manifold
from pglfd.model.gp import GaussianProcess
from pglfd.model.kernel import Kernel

try:
    from pglfd_cpp.kmp_wrapper import get_cpp_kmp

    have_cpp = True
except ImportError:
    have_cpp = False


class KMP(GaussianProcess):
    """KMP on a manifold."""

    def __new__(
        cls,
        manifold_in: Manifold,
        manifold_out: Manifold,
        x: np.ndarray,
        y: np.ndarray,
        kernel: Kernel,
        covariances: np.ndarray,
        lambda1: float = 0.1,
        lambda2: float = 1,
        alpha: float = 1,
    ) -> Any:
        """
        Initialize the KMP.

        :param manifold_in: The input manifold.
        :param manifold_out: The output manifold.
        :param x: `(N, dim_in)` Input data.
        :param y: `(N, dim_out)` Output data.
        :param kernel: The kernel function.
        :param covariances: `(N, dim_out, dim_out)` Covariance matrices.
        :param lambda1: Factor for the mean calculation.
        :param lambda2: Factor for the covariance calculation.
        :param alpha: Scaling factor for final covariance.
        """
        # Check inputs
        if manifold_in.dim_manifold != 1 and x.shape[1] != manifold_in.dim_manifold:
            msg = f"Incorrect input shape {x.shape}, expected (N, {manifold_in.dim_manifold})"
            raise ValueError(msg)
        if manifold_out.dim_manifold != 1 and y.shape[1] != manifold_out.dim_manifold:
            msg = f"Incorrect output shape {y.shape}, expected (N, {manifold_out.dim_manifold})"
            raise ValueError(msg)
        if x.shape[0] != y.shape[0]:
            msg = f"Got {x.shape[0]} inputs but {y.shape[0]} outputs"
            raise ValueError(msg)
        if y.shape[0] != covariances.shape[0]:
            msg = f"Got {covariances.shape[0]} covariances but expected {y.shape[0]}"
            raise ValueError(msg)
        if covariances.shape[1] != covariances.shape[2] or covariances.shape[1] != manifold_out.dim_tangent:
            msg = (
                f"Incorrect covariances shape {covariances.shape}, "
                "expected (N, {manifold_out.dim_tangent}, {manifold_out.dim_tangent})"
            )
            raise ValueError(msg)

        # TODO C++ part for lambda1, alpha
        if have_cpp and cls == KMP:
            kmp_cpp = get_cpp_kmp(manifold_in, manifold_out, x, y, kernel, covariances, lambda1, lambda2, alpha)
            if kmp_cpp is not None:
                return kmp_cpp
        return object.__new__(cls)

    def __getnewargs__(self) -> tuple:
        """
        Get args for pickling.

        :return: manifold_in, manifold_out, mu, x, y, kernel, covariances
        """
        return self.manifold_in, self.manifold_out, self.x, self.y, self.kernel, self.covariances

    def __init__(
        self,
        manifold_in: Manifold,
        manifold_out: Manifold,
        x: np.ndarray,
        y: np.ndarray,
        kernel: Kernel,
        covariances: np.ndarray,
        lambda1: float = 0.1,
        lambda2: float = 1,
        alpha: float = 1,
    ) -> None:
        """
        Initialize the KMP.

        :param manifold_in: The input manifold.
        :param manifold_out: The output manifold.
        :param x: `(N, dim_in)` Input data.
        :param y: `(N, dim_out)` Output data.
        :param kernel: The kernel function.
        :param covariances: `(N, dim_out, dim_out)` Covariance matrices.
        :param lambda1: Factor for the mean calculation.
        :param lambda2: Factor for the covariance calculation.
        :param alpha: Scaling factor for final covariance.
        """
        # basic KMP parameters
        self.covariances = covariances
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.alpha = alpha

        # init nullspace action
        self.ns_actions: list[tuple] = []
        self.x_ns = np.array([])
        self.y_ns = np.array([])
        self.K_hat = np.array([])
        self.K_underbar_inv = np.array([])
        self.ns_use_k_underbar = True
        self.ns_decorrelate = False

        super().__init__(manifold_in, manifold_out, x, y, kernel, 0)

    def append_measurements(self, x: np.ndarray, y: np.ndarray, covariances: np.ndarray) -> None:
        """
        Add measurements.

        :param x: `(dim_in)` or `(N, dim_in)` x value(s).
        :param y: `(dim_out)` or `(N, dim_out)` y value(s).
        :param covariances: `(dim_tangent_out, dim_tangent_out)` or `(N, dim_tangent_out, dim_tangent_out)` covariance(s).
        """
        x = x.reshape(-1, self.manifold_in.dim_manifold)
        y = y.reshape(-1, self.manifold_out.dim_manifold)
        covariances = covariances.reshape(-1, self.manifold_out.dim_tangent, self.manifold_out.dim_tangent)

        if x.shape[0] != y.shape[0] or y.shape[0] != covariances.shape[0]:
            msg = "x, y and covariances need to contain the same number of elements"
            raise ValueError(msg)
        if x.shape[1] != self.manifold_in.dim_manifold:
            msg = "x element size is incorrect"
            raise ValueError(msg)
        if y.shape[1] != self.manifold_out.dim_manifold:
            msg = "y element size is incorrect"
            raise ValueError(msg)
        if covariances.shape[1] != self.manifold_out.dim_tangent or covariances.shape[2] != self.manifold_out.dim_tangent:
            msg = "covariance matrix size is incorrect"
            raise ValueError(msg)

        # update via points
        for i in range(x.shape[0]):
            self.vias[i + self.x.shape[0]] = int(np.argmin(np.linalg.norm(self.manifold_in.log(self.x, x[i]), axis=1)))

        # update data
        insert_idx = self.x.shape[0]
        self.x = np.append(self.x, x, axis=0)
        self.y = np.append(self.y, y, axis=0)
        self.covariances = np.append(self.covariances, covariances, axis=0)
        self._update_k(append_after=insert_idx)

    def remove_measurements(self, measurements_idxs: list[int]) -> None:
        """
        Remove measurements.

        :param measurements_idxs: Indices of measurements to remove.
        """
        # clean inputs
        measurements_idxs = sorted(measurements_idxs)

        # update via points
        for i, measurements_idx in zip(range(len(measurements_idxs)), measurements_idxs, strict=True):
            if measurements_idx in self.vias:
                del self.vias[measurements_idx]
            else:
                for key in list(self.vias.keys()):
                    if self.vias[key] > measurements_idx - i:
                        self.vias[key] -= 1
                    if key > measurements_idx - i:
                        self.vias[key - 1] = self.vias[key]
                        del self.vias[key]

        # update data
        self.x = np.delete(self.x, measurements_idxs, axis=0)
        self.y = np.delete(self.y, measurements_idxs, axis=0)
        self.covariances = np.delete(self.covariances, measurements_idxs, axis=0)
        remove_idxs = measurements_idxs
        self._update_k(remove_idxs=remove_idxs)

    def remove_last_measurement(self) -> None:
        """Remove the last measurement."""
        self.remove_measurements([len(self.x) - 1])

    def set_ns_params(self, *, use_k_underbar: bool, decorrelate: bool) -> None:
        """
        Set ns parameters.

        :param use_k_underbar: Correlate between ns actions.
        :param decorrelate: Decorrelate DoFs.
        """
        self.ns_use_k_underbar = use_k_underbar
        self.ns_decorrelate = decorrelate

    def append_ns_action(self, x: np.ndarray, y: np.ndarray) -> None:
        """
        Append a nullspace action.

        :param x: `(dim_in)` or `(N, dim_in)` x value(s).
        :param y: `(dim_tan)` or `(N, dim_tan)` y value(s).
        """
        x = x.reshape((-1, self.manifold_in.dim_manifold))
        y = y.reshape((-1, self.manifold_out.dim_tangent))
        for i in range(x.shape[0]):
            self.ns_actions.append((x[i], y[i]))
        self._update_ns_action()

    def remove_ns_action(self, remove_idx: list[int]) -> None:
        """
        Remove ns actions at indices.

        :param remove_idx: Nullspace actions to remove.
        """
        self.ns_actions = [self.ns_actions[i] for i in range(len(self.ns_actions)) if i not in remove_idx]
        self._update_ns_action()

    def __repr__(self) -> str:
        """Information about this KMP."""
        return f"<pglfd.model.kmp.KMP on {self.manifold_in!s} / {self.manifold_out!s} with kernel {self.kernel!s}>"

    def _extend_idxs(self, idxs: list) -> list:
        return [
            idx
            for elem in idxs
            for idx in list(range(elem * self.manifold_out.dim_tangent, (elem + 1) * self.manifold_out.dim_tangent))
        ]

    def _update_k(self, append_after: int | None = None, remove_idxs: list | None = None) -> None:
        if append_after is not None:
            # calculate dims
            k_dim = self.x.shape[0] * self.manifold_out.dim_tangent
            append_idx = append_after * self.manifold_out.dim_tangent

            # resize K
            K_copy = self.K.copy()
            self.K = np.zeros((k_dim, k_dim))
            self.K[:append_idx, :append_idx] = K_copy

            # append to K
            self.K[:, append_idx:] = self._apply_kernel(self.x, self.x[append_after:, :])
            self.K[append_idx:, :] = self.K[:, append_idx:].T
        elif remove_idxs is not None:
            # remove elements from K
            all_remove_idxs = self._extend_idxs(remove_idxs)
            self.K = np.delete(self.K, all_remove_idxs, axis=0)
            self.K = np.delete(self.K, all_remove_idxs, axis=1)
        else:
            self.K = self._apply_kernel(self.x, self.x)

        # Prepare the inverse
        # It can be precomputed as it doesn't depend on the query value in a Gaussian process
        self.sigma_ii_inv = np.linalg.inv(self.K + self.lambda1 * self._sigma_block)
        self.sigma_ii_inv2 = np.linalg.inv(self.K + self.lambda2 * self._sigma_block)

        # update the nullspace action in any case
        self._update_ns_action()

    def _update_ns_action(self) -> None:
        if len(self.ns_actions) > 0:
            self.x_ns = np.array([action[0] for action in self.ns_actions]).reshape((-1, self.manifold_in.dim_manifold))
            self.y_ns = np.array([action[1] for action in self.ns_actions]).reshape((-1, self.manifold_out.dim_tangent))
            self.K_hat = self._apply_kernel(self.x, self.x_ns)
            self.K_underbar_inv = np.linalg.inv(self._apply_kernel(self.x_ns, self.x_ns))

    @property
    def _sigma_block(self) -> np.ndarray:
        # create a block diagonal matrix like scipy.linalg.block_diag
        # algorithm suggested by Gemini
        n, dim, _ = self.covariances.shape
        sigma = np.zeros((n, n, dim, dim))
        idx = np.arange(n)
        sigma[idx, idx, :, :] = self.covariances
        return sigma.swapaxes(1, 2).reshape(n * dim, n * dim)

    def _calculate_ns_action(self, x_query: np.ndarray, K_s: np.ndarray) -> np.ndarray:
        """
        Calculate the nullspace action.

        :param x_query: Query value.
        :param K_s: Input kernel.
        :return: The nullspace action to append to delta.
        """
        res = np.zeros(self.manifold_out.dim_tangent)
        if len(self.ns_actions) > 0:
            K_underbar_inv = self.K_underbar_inv if self.ns_use_k_underbar else np.eye(self.K_underbar_inv.shape[0])
            k_hat = self._apply_kernel(x_query.reshape(1, -1), self.x_ns)
            if self.ns_decorrelate:
                for i in range(self.manifold_out.dim_tangent):
                    y_ns = np.zeros_like(self.y_ns)
                    y_ns[:, i] = self.y_ns[:, i]
                    res[i] = ((k_hat - K_s @ self.sigma_ii_inv @ self.K_hat) @ K_underbar_inv @ y_ns.reshape((-1, 1)))[i, 0]
            else:
                res = (k_hat - K_s @ self.sigma_ii_inv @ self.K_hat) @ K_underbar_inv @ self.y_ns.reshape((-1, 1))
        return res.reshape(-1)

    def _calculate_sigma(self, K_ss: np.ndarray, K_s: np.ndarray) -> np.ndarray:
        """
        Calculate the sigma update.

        :param K_ss: Kernel Matrix.
        :param K_s: Kernel Matrix.
        :return: Sigma result (different between Kmp / Gaussian Process).
        """
        return self.alpha * (K_ss - K_s @ self.sigma_ii_inv2 @ K_s.T)
