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

import time
from typing import cast

import numpy as np
from pglfd.manif.product import ProductManifold
from pglfd.model.gaussian import Gaussian
from pglfd.model.gmm import GMM
from pglfd.model.lqt import LQT

from examples.data.letters import get_b_s2, plot_letter_result

# Prepare data
man_out = ProductManifold.from_string("S2")
man_full = ProductManifold.from_string("R1*S2")
t, y, b_s2 = get_b_s2()

# Fit GMM on the full data
data = np.hstack((t.reshape((-1, 1)), y))
gmm, _, _ = GMM.fit(man_full, 10, data)  # type: ignore[assignment]
gmr = gmm.gmr([0], [1])

# Extract 10 via points for LQT
t_lqt = np.linspace(0, 1, 10)
vias = cast(list[Gaussian], gmr(t_lqt))

# Fit coarse LQT: R=1e-4
t0 = time.time()
lqt_coarse = LQT(man_out, y[0, :], t_lqt[1:], vias[1:], dt=0.01, rfactor=1e-4)
it = lqt_coarse.fit(maxit=200, eps=1e-4)
t1 = time.time()
print(f"coarse lqt fit time: {t1 - t0} in {it} iterations")

# Fit medium LQT: R=1e-5
t0 = time.time()
lqt_medium = LQT(man_out, y[0, :], t_lqt[1:], vias[1:], dt=0.01, rfactor=2e-5)
it = lqt_medium.fit(maxit=200, eps=1e-4)
t1 = time.time()
print(f"medium lqt fit time: {t1 - t0} in {it} iterations")

# Fit fine LQT: R=5e-6
t0 = time.time()
lqt_fine = LQT(man_out, y[0, :], t_lqt[1:], vias[1:], dt=0.01, rfactor=5e-6)
it = lqt_fine.fit(maxit=200, eps=1e-4)
t1 = time.time()
print(f"fine lqt fit time: {t1 - t0} in {it} iterations")

# Plot results
plot_letter_result(
    "parametric",
    b_s2,
    ["lqt_coarse", "lqt_medium", "lqt_fine"],
    [lqt_coarse.states, lqt_medium.states, lqt_fine.states],
    vias=vias,
    lqts=[lqt_coarse, lqt_medium, lqt_fine],
)
