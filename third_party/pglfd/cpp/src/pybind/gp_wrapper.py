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


class GaussianProcessWrapper(GaussianRegressorWrapper):
    pass


def get_cpp_gp(
    manifold_in: Manifold, manifold_out: Manifold, x: np.ndarray, y: np.ndarray, kernel: Kernel, sigma: float
) -> GaussianProcessWrapper | None:
    """
    Get a Gaussian process if it exists.

    :param manifold_in: The input manifold.
    :param manifold_out: The output manifold.
    :param x: (N, dim_in) Input data.
    :param y: (N, dim_out) Output data.
    :param kernel: The kernel function.
    :param sigma: Sigma parameter.
    :return: The wrapped C++ Gaussian Process or None.
    """
    from pglfd_cpp import (  # protected import, otherwise might have a partially initialized module
        gaussian,  # noqa: F401
        gaussian_regressor,  # noqa: F401
        gp,
    )

    man_in, man_out, cpp_kernel = get_manifolds_kernel(manifold_in, manifold_out, kernel)

    name_gp = f"GP{man_in}_{man_out}"
    if cpp_kernel is not None and hasattr(gp, name_gp):
        obj = getattr(gp, name_gp)(x, y, cpp_kernel, sigma)
        return GaussianProcessWrapper(manifold_in, manifold_out, obj)
    return None
