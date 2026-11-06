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

from __future__ import annotations

from typing import TYPE_CHECKING

from pglfd_cpp.gaussian_regressor_wrapper import GaussianRegressorWrapper
from pglfd_cpp.kernel_wrapper import get_manifolds_kernel

if TYPE_CHECKING:
    import numpy as np
    from pglfd.manif.manifold import Manifold
    from pglfd.model.kernel import Kernel


class KMPWrapper(GaussianRegressorWrapper):
    @property
    def K(self) -> np.ndarray:
        return self.model.get_K()

    @property
    def sigma_ii_inv(self) -> np.ndarray:
        return self.model.get_sigma_ii_inv()

    @property
    def sigma_ii_inv2(self) -> np.ndarray:
        return self.model.get_sigma_ii_inv2()

    def append_measurements(self, x: np.ndarray, y: np.ndarray, covariances: np.ndarray) -> None:
        x_list = list(x.reshape(-1, self.manifold_in.dim_manifold))
        y_list = list(y.reshape(-1, self.manifold_out.dim_manifold))
        covariances_list = list(covariances.reshape(-1, self.manifold_out.dim_tangent, self.manifold_out.dim_tangent))
        return self.model.append_measurements(x_list, y_list, covariances_list)

    def remove_measurements(self, measurements_idxs: list[int]) -> None:
        return self.model.remove_measurements(measurements_idxs)

    def remove_last_measurement(self) -> None:
        self.model.remove_last_measurement()

    def set_ns_params(self, *, use_k_underbar: bool, decorrelate: bool) -> None:
        self.model.set_ns_params(use_k_underbar, decorrelate)

    def append_ns_action(self, x: np.ndarray, y: np.ndarray) -> None:
        x_list = list(x.reshape(-1, self.manifold_in.dim_manifold))
        y_list = list(y.reshape(-1, self.manifold_out.dim_tangent))
        self.model.append_ns_action(x_list, y_list)

    def remove_ns_action(self, remove_idx: list[int]) -> None:
        self.model.remove_ns_action(remove_idx)


def get_cpp_kmp(
    manifold_in: Manifold,
    manifold_out: Manifold,
    x: np.ndarray,
    y: np.ndarray,
    kernel: Kernel,
    covariances: list[np.ndarray],
    lambda1: float,
    lambda2: float,
    alpha: float,
) -> KMPWrapper | None:
    """
    Get a KMP if it exists.

    :param manifold_in: The input manifold.
    :param manifold_out: The output manifold.
    :param x: (N, dim_in) Input data.
    :param y: (N, dim_out) Output data.
    :param kernel: The kernel function.
    :param covariances: Covariances parameter.
    :param lambda1: Factor for the mean calculation.
    :param lambda2: Factor for the covariance calculation.
    :param alpha: Scaling factor for final covariance.
    :return: A wrapped C++ KMP or None.
    """
    from pglfd_cpp import (  # protected import, otherwise might have a partially initialized module
        gaussian,  # noqa: F401
        gaussian_regressor,  # noqa: F401
        kmp,
    )

    man_in, man_out, cpp_kernel = get_manifolds_kernel(manifold_in, manifold_out, kernel)

    name_kmp = f"KMP{man_in}_{man_out}"
    if cpp_kernel is not None and hasattr(kmp, name_kmp):
        obj = getattr(kmp, name_kmp)(x, y, cpp_kernel, list(covariances), lambda1, lambda2, alpha)
        return KMPWrapper(manifold_in, manifold_out, obj)
    return None
