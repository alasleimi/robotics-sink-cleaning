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

"""GP implementation."""

import typing
from typing import Any

import numpy as np

from pglfd.manif.manifold import Manifold
from pglfd.model.gaussian import Gaussian
from pglfd.model.gaussian_regressor import GaussianRegressor
from pglfd.model.kernel import Kernel

try:
    from pglfd_cpp.gp_wrapper import get_cpp_gp

    have_cpp = True
except ImportError:
    have_cpp = False


class GaussianProcess(GaussianRegressor):
    """Gaussian Process on a manifold."""

    def __new__(
        cls,
        manifold_in: Manifold,
        manifold_out: Manifold,
        x: np.ndarray,
        y: np.ndarray,
        kernel: Kernel,
        sigma: float,
    ) -> Any:
        """
        Initialize the Gaussian process.

        :param manifold_in: The input manifold.
        :param manifold_out: The output manifold.
        :param x: `(N, dim_in)` Input data.
        :param y: `(N, dim_out)` Output data.
        :param kernel: The kernel function.
        :param sigma: Sigma parameter.
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

        if have_cpp and cls == GaussianProcess:
            gp_cpp = get_cpp_gp(manifold_in, manifold_out, x, y, kernel, sigma)
            if gp_cpp is not None:
                return gp_cpp
        return object.__new__(cls)

    def __getnewargs__(self) -> tuple:
        """
        Get args for pickling.

        :return: manifold_in, manifold_out, x, y, kernel, sigma
        """
        return self.manifold_in, self.manifold_out, self.x, self.y, self.kernel, self.sigma

    def __init__(
        self,
        manifold_in: Manifold,
        manifold_out: Manifold,
        x: np.ndarray,
        y: np.ndarray,
        kernel: Kernel,
        sigma: float,
    ) -> None:
        """
        Initialize the Gaussian process.

        :param manifold_in: The input manifold.
        :param manifold_out: The output manifold.
        :param x: `(N, dim_in)` Input data.
        :param y: `(N, dim_out)` Output data.
        :param kernel: The kernel function.
        :param sigma: Sigma parameter.
        """
        self.manifold_in = manifold_in
        self.manifold_out = manifold_out
        self.x = x.reshape(-1, self.manifold_in.dim_manifold)
        self.y = y.reshape(-1, self.manifold_out.dim_manifold)
        self.kernel = kernel
        if self.kernel.is_vectorfield and self.manifold_in.dim_tangent != self.manifold_out.dim_tangent:
            msg = "Cannot use a vector field kernel when input dim != output dim"
            raise ValueError(msg)
        self.sigma = sigma
        self.K = np.array([])

        self.vias: dict[int, int] = {}

        self._update_k()

    @property
    @typing.override
    def input_manifold(self) -> Manifold:
        return self.manifold_in

    @property
    @typing.override
    def output_manifold(self) -> Manifold:
        return self.manifold_out

    def __repr__(self) -> str:
        """Information about the Gaussian Process."""
        return f"<pglfd.model.gp.GaussianProcess on {self.manifold_in!s} / {self.manifold_out!s} with kernel {self.kernel!s}>"

    def _kernel_matrix(self, x1: np.ndarray, x2: np.ndarray, step: int = 1) -> np.ndarray:
        # create selectors as meshgrid
        x = np.array(list(range(x1.shape[0])))
        y = np.array(list(range(x2.shape[0])))
        xx, yy = np.meshgrid(x, y)

        # create meshgrid from inputs
        x1v, x2v = x1[xx.reshape(-1)], x2[yy.reshape(-1)]

        # apply the kernel
        res = self.kernel(x1v, x2v).reshape((-1, step))

        # reorder: we essentially get submatrices stacked on top of each other,
        # reorder to get the desired output matrix
        res = np.block(np.vsplit(res, x2.shape[0]))
        return res.reshape((x1.shape[0] * step, x2.shape[0] * step))

    def _apply_kernel(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        if self.kernel.is_vectorfield:
            return self._kernel_matrix(x1, x2, self.manifold_out.dim_manifold)
        I_O = np.eye(
            self.manifold_out.dim_tangent, self.manifold_out.dim_tangent
        )  # TODO maybe we could use another kernel here?
        return np.kron(self._kernel_matrix(x1, x2), I_O)

    def _update_k(self) -> None:
        self.K = self._apply_kernel(self.x, self.x)

        # Prepare the inverse
        # It can be precomputed as it doesn't depend on the query value in a Gaussian process
        self.sigma_ii_inv = np.linalg.inv(self.K + self._sigma_block)

    @property
    def _sigma_block(self) -> np.ndarray:
        return self.sigma**2 * np.eye(self.K.shape[0])

    def _calculate_sigma(self, K_ss: np.ndarray, K_s: np.ndarray) -> np.ndarray:
        """
        Calculate the sigma update.

        :param K_ss: Kernel Matrix.
        :param K_s: Kernel Matrix.
        :return: Sigma result (different between Kmp / Gaussian Process).
        """
        return 1 * (K_ss - K_s @ self.sigma_ii_inv @ K_s.T)

    def _calculate_ns_action(self, x_query: np.ndarray, K_s: np.ndarray) -> np.ndarray | float:
        """
        Calculate the nullspace action.

        :param x_query: Query value.
        :param K_s: Input kernel.
        :return: The nullspace action to append to delta.
        """
        _ = (x_query, K_s)
        return 0

    def __call__(self, x_query: np.ndarray) -> Gaussian | list[Gaussian]:
        """
        Query the Gaussian process for some output.

        :param x_query: `(N, dim_in)` or `(dim_in)` The query value.
        :return: The calculated output.
        """
        inp_shape = x_query.shape
        x_query = x_query.reshape(-1, self.manifold_in.dim_manifold)

        # Calculate input log
        orig_log = self.manifold_out.log(self.manifold_out.id_elem, self.y)
        log_x_in, discontinuities = self.manifold_out.continuous_log(orig_log.copy(), vias=self.vias, start_low=False)
        olog_x_in, odiscontinuities = self.manifold_out.continuous_log(orig_log.copy(), vias=self.vias, start_low=True)

        res = []
        for x_i in x_query:
            K_s = self._apply_kernel(x_i.reshape(1, -1), self.x)
            K_ss = self._apply_kernel(x_i.reshape(1, -1), x_i.reshape(1, -1))

            # Prepare input-dependent data
            mu = self.manifold_out.id_elem

            # Find closest point
            # TODO verify that we didn't hit a via
            closest = int(np.argmin(np.linalg.norm(self.manifold_in.log(self.x, x_i), axis=1)))
            closest_via = None
            if len(self.vias.keys()) > 0:
                closest_via = list(self.vias)[np.argmin(closest - np.array([self.vias[key] for key in self.vias]))]
            if closest in self.vias:
                closest = self.vias[closest]

            log_x_in_use = log_x_in
            if len(discontinuities) > 0 or len(self.vias.keys()) > 0:
                disc_cond = len(discontinuities) > 0 and (
                    len(odiscontinuities) == 0
                    or np.min(np.abs(closest - np.array(odiscontinuities)))
                    > np.min(np.abs(closest - np.array(discontinuities)))
                )
                via_disc_cond = True
                via_cond = False
                if closest_via is not None:
                    via_disc_cond = len(odiscontinuities) == 0 or np.min(
                        np.abs(closest - np.array(odiscontinuities))
                    ) > np.abs(closest - self.vias[closest_via])
                    via_cond = np.linalg.norm(orig_log[closest_via] - olog_x_in[closest_via]) < np.linalg.norm(
                        orig_log[closest_via] - log_x_in[closest_via]
                    ) and (
                        len(odiscontinuities) == 0
                        or np.min(np.abs(closest - np.array(odiscontinuities))) > np.abs(closest - self.vias[closest_via])
                    )

                if (disc_cond and via_disc_cond) or via_cond:
                    log_x_in_use = olog_x_in

            # Calculate update
            delta = -(K_s @ self.sigma_ii_inv @ log_x_in_use.reshape(-1, 1)).reshape(-1) + self._calculate_ns_action(x_i, K_s)
            mu = self.manifold_out.exp(delta, mu)

            # Calculate sigma
            sigma = self._calculate_sigma(K_ss, K_s)

            # Append
            res.append(Gaussian(self.manifold_out, mu, sigma))

        # reshape
        if len(inp_shape) <= 1 and len(res) == 1:
            return res[0]
        return res
