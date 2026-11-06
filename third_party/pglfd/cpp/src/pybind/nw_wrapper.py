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
from pglfd_cpp.kernel_wrapper import get_scalar_kernel

if TYPE_CHECKING:
    import numpy as np
    from pglfd.manif.manifold import Manifold
    from pglfd.model.kernel import Kernel
    from pglfd_cpp.gaussian_wrapper import GaussiansListWrapper, GaussianWrapper


class NadarayaWatsonWrapper(GaussianRegressorWrapper):
    def __call__(
        self, x_query: np.ndarray, reg: float = 1e-8, eps: float = 1e-8, maxit: int = 50
    ) -> GaussiansListWrapper | GaussianWrapper:
        return super().__call__(x_query, reg, eps, maxit)


def get_cpp_nw(
    manifold_in: Manifold, manifold_out: Manifold, x: np.ndarray, y: np.ndarray, kernel: Kernel
) -> NadarayaWatsonWrapper | None:
    """
    Get a Nadaraya-Watson regressor if it exists.

    :param manifold_in: The input manifold.
    :param manifold_out: The output manifold.
    :param x: (N, dim_in) Input data.
    :param y: (N, dim_out) Output data.
    :param kernel: The kernel function.
    :return: A wrapped C++ Nadaraya-Watson regressor or None.
    """
    from pglfd_cpp import (  # protected import, otherwise might have a partially initialized module
        gaussian,  # noqa: F401
        gaussian_regressor,  # noqa: F401
        nw,
    )

    man_in, cpp_kernel = get_scalar_kernel(manifold_in, kernel)
    man_out = "".join(str(manifold_out).split("*"))

    name_nw = f"NadarayaWatson{man_in}_{man_out}"
    if cpp_kernel is not None and hasattr(nw, name_nw):
        obj = getattr(nw, name_nw)(x, y, cpp_kernel)
        return NadarayaWatsonWrapper(manifold_in, manifold_out, obj)
    return None
