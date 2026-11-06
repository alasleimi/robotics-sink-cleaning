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

"""Nadaraya-Watson on manifold implementation :cite:p:`muehlbauer2026geometric`."""

import typing
from typing import Any

import numpy as np

from pglfd.manif.manifold import Manifold
from pglfd.model.gaussian import Gaussian
from pglfd.model.gaussian_regressor import GaussianRegressor
from pglfd.model.kernel import Kernel

try:
    from pglfd_cpp.nw_wrapper import get_cpp_nw

    have_cpp = True
except ImportError:
    have_cpp = False


class NadarayaWatson(GaussianRegressor):
    """Nadaraya-Watson regression on a manifold."""

    def __new__(
        cls,
        manifold_in: Manifold,
        manifold_out: Manifold,
        x: np.ndarray,
        y: np.ndarray,
        kernel: Kernel,
    ) -> Any:
        """
        Initialize the Nadaraya-Watson regression.

        :param manifold_in: The input manifold.
        :param manifold_out: The output manifold.
        :param x: `(N, dim_in)` Input data.
        :param y: `(N, dim_out)` Output data.
        :param kernel: The kernel function.
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

        if have_cpp and cls == NadarayaWatson:
            nw_cpp = get_cpp_nw(manifold_in, manifold_out, x, y, kernel)
            if nw_cpp is not None:
                return nw_cpp
        return object.__new__(cls)

    def __getnewargs__(self) -> tuple:
        """
        Get args for pickling.

        :return: manifold_in, manifold_out, x, y, kernel
        """
        return self.manifold_in, self.manifold_out, self.x, self.y, self.kernel

    def __init__(
        self,
        manifold_in: Manifold,
        manifold_out: Manifold,
        x: np.ndarray,
        y: np.ndarray,
        kernel: Kernel,
    ) -> None:
        """
        Initialize the Nadaraya-Watson regression.

        :param manifold_in: The input manifold.
        :param manifold_out: The output manifold.
        :param x: `(N, dim_in)` Input data.
        :param y: `(N, dim_out)` Output data.
        :param kernel: The kernel function.
        """
        self.manifold_in = manifold_in
        self.manifold_out = manifold_out
        self.x = x.reshape(-1, self.manifold_in.dim_manifold)
        self.y = y.reshape(-1, self.manifold_out.dim_manifold)
        self.kernel = kernel
        if self.kernel.is_vectorfield:
            msg = "Can only use scalar kernels!"
            raise ValueError(msg)

    @property
    @typing.override
    def input_manifold(self) -> Manifold:
        return self.manifold_in

    @property
    @typing.override
    def output_manifold(self) -> Manifold:
        return self.manifold_out

    def __call__(
        self, x_query: np.ndarray, reg: float = 1e-8, eps: float = 1e-8, maxit: int = 50
    ) -> Gaussian | list[Gaussian]:
        """
        Query the Nadaraya-Watson regressor for some output.

        :param x_query: `(N, dim_man)` or `(dim_man)` The input value.
        :param reg: Regularization constant for the covariance estimation.
        :param eps: Epsilon value for the frechet mean.
        :param maxit: Maximum number of iterations to perform before exiting.
        :return: `(N, dim_man)` or `(dim_man)` The calculated output.
        """
        inp_shape = x_query.shape
        x_query = x_query.reshape(-1, self.manifold_in.dim_manifold)

        res = []
        for x_i in x_query:
            weights = self.kernel(x_i, self.x)
            weights /= np.sum(weights)
            res.append(Gaussian.mle_from_data(self.manifold_out, self.y, weights, reg, eps, maxit))

        # reshape
        if len(inp_shape) <= 1 and len(res) == 1:
            return res[0]
        return res
