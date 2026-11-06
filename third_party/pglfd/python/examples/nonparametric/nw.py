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
from pglfd.model.nw import NadarayaWatson

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

# Extract 15 via points for NW
t_nw = np.linspace(0, 1, 15)
res = cast(list[Gaussian], gmr(t_nw))
y_nw = np.array([gauss.mu for gauss in res])

# Fit Nadaraya-Watson regressor with kernel with l=0.01
kernel_01 = RBFKernel(man_in, length_scale=0.01)
nw_01 = NadarayaWatson(man_in, man_out, t_nw, y_nw, kernel_01)
nwres_01 = nw_01(t_pred)

# Fit Nadaraya-Watson regressor with kernel with l=0.05
kernel_05 = RBFKernel(man_in, length_scale=0.05)
nw_05 = NadarayaWatson(man_in, man_out, t_nw, y_nw, kernel_05)
nwres_05 = nw_05(t_pred)

# Fit Nadaraya-Watson regressor with kernel with l=0.1
kernel_1 = RBFKernel(man_in, length_scale=0.1)
nw_1 = NadarayaWatson(man_in, man_out, t_nw, y_nw, kernel_1)
nwres_1 = nw_1(t_pred)

# Plot results
plot_letter_result(
    "nonparametric",
    b_s2,
    ["nw_01", "nw_05", "nw_1"],
    [nwres_01, nwres_05, nwres_1],
    vias=res,
)
