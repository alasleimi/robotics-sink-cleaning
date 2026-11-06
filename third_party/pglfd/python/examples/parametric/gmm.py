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

from types import SimpleNamespace

import numpy as np
from pglfd.manif.product import ProductManifold
from pglfd.model.gaussian import gaussian_product
from pglfd.model.gmm import GMM

from examples.data.letters import get_b_s2, plot_letter_result

# Prepare data
man_full = ProductManifold.from_string("R1*S2")
t, y, b_s2 = get_b_s2()
data = np.hstack((t.reshape((-1, 1)), y))

# Fit fine GMM (10 Gaussians)
gmm, _, _ = GMM.fit(man_full, 10, data)  # type: ignore[assignment]

# Draw 50 samples from the GMM
samples = gmm.sample(50)

# Approximate single Gaussian of marginalized Gaussians 3, 4
mixture_gaussian = GMM(gmm.gaussians[3:5]).approximate_single_gaussian(
    np.array([1.0, 1.0])
)

# Gaussian product of Gaussians 3, 4
product_gaussian = gaussian_product(*gmm.gaussians[3:5])

# Plot results
plot_letter_result(
    "parametric",
    b_s2,
    ["gmm_fit", "gmm_sample", "mixture_gaussian", "product_gaussian"],
    None,
    gmms=[
        gmm,
        gmm,
        SimpleNamespace(gaussians=gmm.gaussians[3:5] + [mixture_gaussian]),
        SimpleNamespace(gaussians=gmm.gaussians[3:5] + [product_gaussian]),
    ],
    samples=[None, samples, None, None],
)
