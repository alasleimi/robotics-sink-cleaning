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

from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from pglfd.manif.manifold import Manifold
    from pglfd.model.kernel import Kernel


def get_scalar_kernel(manifold_in: Manifold, kernel: Kernel) -> tuple[Manifold, Any] | tuple[Manifold, None]:
    from pglfd_cpp import (
        kernel as cpp_kernel,  # protected import, otherwise might have a partially initialized module
    )

    # Get the manifolds
    man_in = "".join(str(manifold_in).split("*"))

    # First get the kernel
    cpp_kernel_sel = None
    from pglfd.model.kernel import MaternKernel, RBFKernel

    if isinstance(kernel, RBFKernel):
        name_kernel = f"RBF{man_in}"
        if hasattr(cpp_kernel, name_kernel):
            length_scale = np.ones(manifold_in.dim_tangent) * kernel.length_scale
            cpp_kernel_sel = getattr(cpp_kernel, name_kernel)(length_scale, kernel.height_scale)
    elif isinstance(kernel, MaternKernel):
        name_kernel = f"MATERNP{kernel.p}_{man_in}"
        if hasattr(cpp_kernel, name_kernel):
            length_scale = np.ones(manifold_in.dim_tangent) * kernel.length_scale
            cpp_kernel_sel = getattr(cpp_kernel, name_kernel)(length_scale, kernel.height_scale)

    return man_in, cpp_kernel_sel


def get_manifolds_kernel(
    manifold_in: Manifold, manifold_out: Manifold, kernel: Kernel
) -> tuple[Manifold, Manifold, Any] | tuple[Manifold, Manifold, None]:
    from pglfd_cpp import (
        kernel as cpp_kernel,  # protected import, otherwise might have a partially initialized module
    )

    # First get the kernel
    cpp_kernel_sel = None
    from pglfd.model.kernel import (
        MaternKernelDiagonal,
        RBFKernelDiagonal,
    )

    # Get the manifolds
    man_in, cpp_kernel_scalar = get_scalar_kernel(manifold_in, kernel)
    man_out = "".join(str(manifold_out).split("*"))

    if cpp_kernel_scalar:
        name_kernel = f"KERNEL_PRODUCT{man_in}_{man_out}"
        if hasattr(cpp_kernel, name_kernel):
            cpp_kernel_sel = getattr(cpp_kernel, name_kernel)(cpp_kernel_scalar)
    elif isinstance(kernel, RBFKernelDiagonal):
        name_kernel = f"RBF_DIAG{man_in}_{man_out}"
        if hasattr(cpp_kernel, name_kernel):
            length_scale_rbf = np.ones(manifold_in.dim_tangent) * kernel.length_scale_rbf
            length_scale_diag = np.ones(manifold_in.dim_tangent) * kernel.length_scale_diag
            cpp_kernel_sel = getattr(cpp_kernel, name_kernel)(length_scale_rbf, length_scale_diag, kernel.height_scale)
    elif isinstance(kernel, MaternKernelDiagonal):
        name_kernel = f"MATERNP{kernel.p}_DIAG{man_in}_{man_out}"
        if hasattr(cpp_kernel, name_kernel):
            length_scale_matern = np.ones(manifold_in.dim_tangent) * kernel.length_scale_matern
            cpp_kernel_sel = getattr(cpp_kernel_sel, name_kernel)(
                length_scale_matern, kernel.length_scales_diag, kernel.height_scale
            )

    return man_in, man_out, cpp_kernel_sel
