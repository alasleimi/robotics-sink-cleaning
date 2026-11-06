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

from pglfd.manif.product import ProductManifold
from pglfd.model.gaussian import Gaussian

from examples.data.letters import get_b_s2, plot_letter_result

# Prepare data
man_s2 = ProductManifold.from_string("S2")
_, _, b_s2 = get_b_s2()
b_s2 = b_s2[:, :, 70:100]
data = b_s2.transpose((2, 0, 1)).reshape((-1, 3))

# Fit Gaussian
gaussian = Gaussian.mle_from_data(man_s2, data)  # type: ignore[assignment]

# Calculate the probability of some trajectory points
for sample in b_s2[0, :, ::5].transpose((1, 0)).reshape((-1, 3)):
    print(f"Probability at {sample}:\t{gaussian.probability(sample)[0]}")

# Draw 50 samples from the Gaussian
samples = gaussian.sample(50)

# Plot results
plot_letter_result(
    "gaussian",
    b_s2,
    ["gaussian", "gaussian_sample"],
    None,
    gaussians=[gaussian, gaussian],
    samples=[None, samples],
)
