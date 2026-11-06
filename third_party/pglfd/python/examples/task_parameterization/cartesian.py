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
from pglfd.geom.se3 import invert_transformation, multiply_poses
from pglfd.manif.product import ProductManifold
from pglfd.model.gaussian import Gaussian
from pglfd.model.gmm import GMM
from pglfd.model.kernel import MaternKernel
from pglfd.model.kmp import KMP
from pglfd.model.tp import TaskParameterizedModel

from examples.data.trajectory import get_trajectories, plot_tp_model

# manifolds
man_in = ProductManifold.from_string("R1")
man_out = ProductManifold.from_string("R3*SO3")
man_full = ProductManifold.from_string("R1*R3*SO3")  # time * pose

# prepare data: transform into their respective task frames
trajectories: list[list] = [[], []]
for i in range(len(trajectories)):
    for poses, trajectory in get_trajectories():
        # transform the pose part into the respective frame, keep time unchanged
        trajectory[:, 1:] = multiply_poses(
            invert_transformation(poses[i]), trajectory[:, 1:]
        )
        trajectories[i].append(trajectory[::10, :])

# fit the GMMs
gmms = []
gmrs = []
for trajectory in trajectories:
    # Fit GMM (10 Gaussians)
    gmm, _, _ = GMM.fit(man_full, 10, np.vstack(trajectory))  # type: ignore[assignment]
    gmms.append(gmm)
    gmrs.append(gmm.gmr([0], [1, 2]))

# fit the KMPs
t_fit = np.linspace(0, 1, 100)
kmps = []
for gmr in gmrs:
    gmrres = cast("list[Gaussian]", gmr(t_fit))
    kernel = MaternKernel(man_in, 0.3)
    kmps.append(
        KMP(
            man_in,
            man_out,
            t_fit,
            np.array([gauss.mu for gauss in gmrres]),
            kernel,
            np.array([gauss.sigma for gauss in gmrres]),
        )
    )

# create a tp model for the 0° case
origins = [
    (
        "start_model",
        np.array([0.54, 0.77, -0.06, -0.00870816, 0.99976146, -0.01647045, 0.01139969]),
    ),
    (
        "end_model",
        np.array([0.52, 0.21, -0.07, -0.01256791, 0.9591583, -0.28249988, 0.00715961]),
    ),
]
tp_model = TaskParameterizedModel(origins, kmps)

# plot the tp model for the 0° case
plot_tp_model("cartesian", tp_model)


# create a tp model for the 90° case
origins = [
    (
        "start_model",
        np.array([0.54, 0.77, -0.06, -0.00870816, 0.99976146, -0.01647045, 0.01139969]),
    ),
    (
        "end_model",
        np.array([0.42, 0.21, -0.07, -0.69407946, -0.64914405, 0.23067316, 0.20893916]),
    ),
]
tp_model = TaskParameterizedModel(origins, kmps)

# plot the tp model for the 90° case
plot_tp_model("cartesian_90deg", tp_model)
