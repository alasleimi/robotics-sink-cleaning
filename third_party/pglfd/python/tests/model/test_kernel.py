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

import numpy as np
from sklearn.gaussian_process.kernels import Matern as SklearnMatern

from pglfd.manif.product import ProductManifold
from pglfd.model.gp import GaussianProcess
from pglfd.model.kernel import MaternKernel, MaternKernelDiagonal, RBFKernel, RBFKernelDiagonal
from tests.model.test_data import generate_data_sine


def test_kernels_matern() -> None:
    # Prepare
    man = ProductManifold.from_string("R1*R3*SO3")
    data = generate_data_sine(20)

    def matern_p1_fun(x1, x2):
        length_scale = 0.1
        dist = man.log(x2, x1).reshape((-1, man.dim_tangent, 1))
        r = np.sqrt(dist.transpose((0, 2, 1)) @ dist).reshape(-1)
        return (1 + np.sqrt(3) * r / length_scale) * np.exp(-np.sqrt(3) * r / length_scale)

    def matern_p2_fun(x1, x2):
        length_scale = 0.1
        dist = man.log(x2, x1).reshape((-1, man.dim_tangent, 1))
        r = np.sqrt(dist.transpose((0, 2, 1)) @ dist).reshape(-1)
        return (1 + np.sqrt(5) * r / length_scale + 5 * r * r / (3 * length_scale * length_scale)) * np.exp(
            -np.sqrt(5) * r / length_scale
        )

    # results
    obj = type("", (), {})()
    obj.manifold_in = man
    obj.kernel = matern_p1_fun
    matern_p1_baseline = GaussianProcess._kernel_matrix(obj, data, data)
    obj.kernel = matern_p2_fun
    matern_p2_baseline = GaussianProcess._kernel_matrix(obj, data, data)
    obj.kernel = MaternKernel(man, p=1)
    matern_p1 = GaussianProcess._kernel_matrix(obj, data, data)
    obj.kernel = MaternKernel(man, p=2)
    matern_p2 = GaussianProcess._kernel_matrix(obj, data, data)

    assert np.allclose(matern_p1_baseline, matern_p1)
    assert np.allclose(matern_p2_baseline, matern_p2)

    # compare with sklearn
    man = ProductManifold.from_string("R1*R3")
    obj.manifold_in = man
    obj.kernel = MaternKernel(man, p=1)
    matern_p1 = GaussianProcess._kernel_matrix(obj, data[:, :4], data[:, :4])
    matern_p1_sklearn = SklearnMatern(length_scale=0.1, nu=1.5)
    matern_p1_sklearn = matern_p1_sklearn(data[:, :4], data[:, :4])
    obj.kernel = MaternKernel(man, p=2)
    matern_p2 = GaussianProcess._kernel_matrix(obj, data[:, :4], data[:, :4])
    matern_p2_sklearn = SklearnMatern(length_scale=0.1, nu=2.5)
    matern_p2_sklearn = matern_p2_sklearn(data[:, :4], data[:, :4])
    assert np.allclose(matern_p1_sklearn, matern_p1)
    assert np.allclose(matern_p1_sklearn, matern_p1)


def test_kernels() -> None:
    # Prepare
    man_full = ProductManifold.from_string("R1*R3*SO3")
    data_full = generate_data_sine(20)

    for man_sel in [[0], [0, 1, 2]]:
        man, idx_man, _ = man_full.submanifold(man_sel)
        data = data_full[:, idx_man]
        kernels = {
            "rbf": RBFKernel(man),
            "matern_p0": MaternKernel(man, p=0),
            "matern_p1": MaternKernel(man, p=1),
            "matern_p2": MaternKernel(man, p=2),
        }
        if man.dim_tangent > 1:
            kernels["rbf_diag"] = RBFKernelDiagonal(man)
            kernels["matern_diag"] = MaternKernelDiagonal(man, p=1)
        for kernel_name in kernels:
            kernel = kernels[kernel_name]
            for scaler_x1 in [1, 2, 4]:
                for scaler_x2 in [1, 2, 4]:
                    x1_sel = int(data.shape[0] / scaler_x1)
                    x2_sel = int(data.shape[0] / scaler_x2)

                    # scalar result
                    step = man.dim_tangent if kernel.is_vectorfield else 1
                    kernel_matrix = np.zeros((x1_sel * step, x2_sel * step))
                    for i in range(x1_sel):
                        for j in range(x2_sel):
                            kernel_matrix[i * step : (i + 1) * step, j * step : (j + 1) * step] = kernel(data[i], data[j])

                    # vectorized result
                    obj = type("", (), {})()
                    obj.manifold_in = man
                    obj.kernel = kernel
                    vec_result = GaussianProcess._kernel_matrix(obj, data[:x1_sel, :], data[:x2_sel, :], step)

                    # compare
                    assert np.allclose(kernel_matrix, vec_result), f"{kernel_name} is not properly vectorized"
                    if kernel_matrix.shape[0] == kernel_matrix.shape[1] and "matern" not in kernel_name:
                        np.linalg.cholesky(kernel_matrix)
                        np.linalg.cholesky(vec_result)
