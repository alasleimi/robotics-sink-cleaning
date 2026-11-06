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
GMR implementation.

:cite:t:`zeestraten2017manifold`
:cite:t:`calinon2020gaussians`.
"""

import typing
from typing import cast

import numpy as np

from pglfd.manif.manifold import Manifold
from pglfd.manif.product import ProductManifold
from pglfd.model.gaussian import Gaussian
from pglfd.model.gaussian_regressor import GaussianRegressor
from pglfd.model.gmm import GMM


class GMR(GaussianRegressor):
    """Gaussian Mixture Regression on a manifold [1-3]."""

    def __init__(self, gmm: GMM, idx_in: list, idx_out: list) -> None:
        """
        Create a Gaussian Mixture Regression class.

        :param gmm: The trained gaussian mixture model.
        :param idx_in: List of input manifold idx's.
        :param idx_out: List of output manifold idx's.
        :return: GMR which can be queried with input values.
        """
        # Input and output manifolds
        if type(gmm.gaussians[0].manifold).__name__ != "ProductManifold":
            msg = "GMR only works for product manifolds!"
            raise TypeError(msg)
        self.manifold_in, _idx_manifold_in, _idx_tangent_in = cast(ProductManifold, gmm.gaussians[0].manifold).submanifold(
            idx_in
        )
        self.manifold_out, _idx_manifold_out, _idx_tangent_out = cast(ProductManifold, gmm.gaussians[0].manifold).submanifold(
            idx_out
        )

        # Marginalize whole gmm
        self.marg_gmm = gmm.margin(idx_in)

        # Conditioning and marginalization lists
        self.gauss_cond_list = []
        self.gauss_marg_list = []
        for i in range(len(gmm.gaussians)):
            self.gauss_cond_list.append(gmm.gaussians[i].condition(idx_in, idx_out))
            self.gauss_marg_list.append(gmm.gaussians[i].margin(idx_out))

        # User info status
        self.had_runtime_warning = False

    @property
    @typing.override
    def input_manifold(self) -> Manifold:
        return self.manifold_in

    @property
    @typing.override
    def output_manifold(self) -> Manifold:
        return self.manifold_out

    def __call__(
        self, x_query: np.ndarray, eps_marg: float = 1e-8, eps_cond: float = 1e-8, eps_approx: float = 1e-8, maxit: int = 50
    ) -> Gaussian | list[Gaussian]:
        """
        Query GMR for some output.

        :param x_query: `(N, dim_man)` or `(dim_man)` The input value.
        :param eps_marg: When to use marginalization vs gaussian conditioning.
        :param eps_cond: Stopping criterion for gaussian conditioning.
        :param eps_approx: Stopping criterion for single gaussian approximation.
        :param maxit: Maximum number of iterations to perform before exiting.
        :return: `(N, dim_man)` or `(dim_man)` The calculated output.
        """
        # Reshape input
        shape_in = x_query.shape
        x_query = x_query.reshape((-1, self.manifold_in.dim_manifold))

        # Compute weights
        h = self.marg_gmm.expectation(x_query)
        h /= np.sum(h, axis=1, keepdims=True)

        # Loop over inputs
        gmr_result = []
        for i, x_i in zip(range(x_query.shape[0]), x_query, strict=True):
            # Initialize "new gmm" using either the marginalization result
            # or the conditioning result, depending on the weight factor.
            gauss_selection = []
            for j in range(h.shape[1]):
                use_cond = h[i, j] >= eps_marg
                if use_cond:
                    try:
                        gauss_selection.append(self.gauss_cond_list[j](x_i, eps=eps_cond, maxit=maxit))
                    except RuntimeWarning:
                        if not self.had_runtime_warning:
                            print(
                                "Warning: Gaussian condition did not converge, using simple marginalization [print only once]"
                            )
                            self.had_runtime_warning = True
                        use_cond = False

                if not use_cond:
                    gauss_selection.append(self.gauss_marg_list[j])

            flat_gaussians = cast("list[Gaussian]", gauss_selection)
            gmr_result.append(GMM(flat_gaussians).approximate_single_gaussian(h[i], eps=eps_approx, maxit=maxit))

        # reshape
        if len(shape_in) <= 1 and x_query.shape[0] == 1:
            return gmr_result[0]
        return gmr_result
