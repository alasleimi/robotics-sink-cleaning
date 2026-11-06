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

from typing import cast

import numpy as np
from pglfd.manif.product import ProductManifold
from pglfd.model.gaussian import Gaussian
from pglfd.model.gmm import GMM
from pglfd.model.kernel import RBFKernel
from pglfd.model.kmp import KMP

from examples.data.letters import get_b_s2, plot_letter_result

# Prepare data
man_in = ProductManifold.from_string("R1")
man_out = ProductManifold.from_string("S2")
man_full = ProductManifold.from_string("R1*S2")
t, y, b_s2 = get_b_s2()

# Fit GMM on the full data
data = np.hstack((t.reshape((-1, 1)), y))
gmm, _, _ = GMM.fit(man_full, 10, data)  # type: ignore[assignment]
gmr = gmm.gmr([0], [1])
t_pred = np.linspace(0, 1, 101)

# Extract 15 via points for GP
t_nw = np.linspace(0, 1, 15)
res = cast(list[Gaussian], gmr(t_nw))
y_kmp = np.array([gauss.mu for gauss in res])
cov_kmp = np.array([gauss.sigma for gauss in res])

# Fit Gaussian process with kernel with l=0.01
kernel_01 = RBFKernel(man_in, length_scale=0.01)
kmp_01 = KMP(man_in, man_out, t_nw, y_kmp, kernel_01, cov_kmp)
kmpres_01 = kmp_01(t_pred)

# Fit Gaussian process with kernel with l=0.05
kernel_05 = RBFKernel(man_in, length_scale=0.05)
kmp_05 = KMP(man_in, man_out, t_nw, y_kmp, kernel_05, cov_kmp)
kmpres_05 = kmp_05(t_pred)

# Fit Gaussian process with kernel with l=0.1
kernel_1 = RBFKernel(man_in, length_scale=0.1)
kmp_1 = KMP(man_in, man_out, t_nw, y_kmp, kernel_1, cov_kmp)
kmpres_1 = kmp_1(t_pred)

# Plot results
plot_letter_result(
    "nonparametric",
    b_s2,
    ["kmp_01", "kmp_05", "kmp_1"],
    [kmpres_01, kmpres_05, kmpres_1],
    plot_covs=[False, True, True],
    vias=res,
)
