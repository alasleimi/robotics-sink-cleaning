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
from pglfd.model.gaussian import Gaussian, gaussian_product

from examples.data.letters import get_b_s2, plot_letter_result

# Prepare data
man_s2 = ProductManifold.from_string("S2")
_, _, b_s2 = get_b_s2()
first_b_s2 = b_s2[:, :, 20:50]
first_data = first_b_s2.transpose((2, 0, 1)).reshape((-1, 3))
second_b_s2 = b_s2[:, :, 100:130]
second_data = second_b_s2.transpose((2, 0, 1)).reshape((-1, 3))

# Fit Gaussian
first_gaussian = Gaussian.mle_from_data(man_s2, first_data)  # type: ignore[assignment]
second_gaussian = Gaussian.mle_from_data(man_s2, second_data)  # type: ignore[assignment]

# Calculate the Gaussian product
product_gaussian = gaussian_product(first_gaussian, second_gaussian)

# Plot results
plot_letter_result(
    "gaussian",
    np.vstack((first_b_s2, second_b_s2)),
    ["gaussian_product"],
    None,
    gmms=[
        SimpleNamespace(gaussians=[first_gaussian, second_gaussian, product_gaussian])
    ],
)
