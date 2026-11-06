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

import os
import pickle
from typing import Any, cast

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import yaml
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pglfd.geom.so3 import quaternion_to_rotmat
from pglfd.manif.product import ProductManifold
from pglfd.model.gaussian import Gaussian
from pglfd.model.gmm import GMM
from pglfd.model.gmr import GMR
from pglfd.model.tp import TaskParameterizedModel
from pglfd.visualization.matplotlib import plot_coordinate_system
from pglfd.visualization.util import ellipsoid_plane_intersection_points, merge_shapes

mpl.rcParams["pdf.fonttype"] = 42
mpl.rcParams["ps.fonttype"] = 42


def get_trajectory_gmr() -> tuple[GMM, GMR]:
    """
    Get the trajectory GMR.

    :return: The trajectory GMR.
    """
    # load data
    with open(os.path.join(os.path.dirname(__file__), "gmm_trajectory.yaml"), "r") as f:
        data = yaml.safe_load(f.read())

    # fit the gmm
    man = ProductManifold.from_string("R1*R3*SO3")
    gaussians = [
        Gaussian(man, np.array(gauss["mu"]), np.array(gauss["sigma"]))
        for gauss in data["gaussians"]
    ]
    gmm = GMM(gaussians, np.array(data["priors"]))

    # return
    return gmm, gmm.gmr([0], [1, 2])


def get_trajectories() -> list:
    """
    Load the demonstrations and their frames.

    :return: List of trajectories and task frames.
    """

    demo_dir = os.path.join(os.path.dirname(__file__), "demos")
    trajectories = []
    for file_name in sorted(os.listdir(demo_dir)):
        # skip 180° case
        if "p4" in file_name:
            continue

        with open(os.path.join(demo_dir, file_name), "rb") as f:
            data = pickle.load(f)

        trajectory = np.array(data["trajectory"][0])
        poses = np.array(data["poses"][0])

        trajectories.append((poses, trajectory))

    return trajectories


def _plot_prob_traj(
    ax: Any, label: str, gaussians: list[Gaussian], gauss_dist: float = 0.05
) -> None:
    # plot mean
    poses = np.array([gauss.mu for gauss in gaussians])
    ax.plot(poses[:, 0], poses[:, 1], poses[:, 2], label=label)

    # extract direction vector
    n_vec = []
    for i in range(poses.shape[0] - 1):
        n_vec.append(poses[i + 1, :3] - poses[i, :3])
    n_vec.append(poses[-1, :3] - poses[-2, :3])

    # plot covariance
    ellipse_points = []
    for i, gauss in enumerate(gaussians):
        ellipse_points.append(
            ellipsoid_plane_intersection_points(
                poses[i, :3],
                gauss.sigma[:3, :3],
                n_vec[i],
                n_points=100,
            ).T
        )

    # merge all ellipses + plot
    faces = merge_shapes(ellipse_points, add_sides=True)
    poly = Poly3DCollection(faces, facecolors=[0.5, 0.5, 0.5, 0.3])
    ax.add_collection3d(poly)

    # plot axes for orientation
    last_mean = np.array([-1, -1, -1])
    for gauss in gaussians:
        if np.linalg.norm(last_mean[:3] - gauss.mu[:3]) > gauss_dist:
            plot_coordinate_system(ax, gauss.mu, scaling=0.04, labels=None)
            last_mean = gauss.mu


def plot_tp_model(name: str, tp_model: TaskParameterizedModel) -> None:
    # update dir
    img_dir = os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "docs",
        "algorithms",
        "images",
        "tp",
    )

    # create axis
    ax = plt.figure(name).add_subplot(projection="3d")

    # pane colors
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor("w")
    ax.yaxis.pane.set_edgecolor("w")
    ax.zaxis.pane.set_edgecolor("w")

    # limits
    ax.set_xlim([-0.0, 0.9])
    ax.set_ylim([-0.0, 0.9])
    ax.set_zlim([-0.3, 0.6])

    # axis labels
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    # view
    ax.view_init(elev=35.0, azim=-20)

    # plot models
    for model, origin in zip(tp_model.models, tp_model.model_origins):
        # Query the model
        gaussians = cast(list[Gaussian], model(np.linspace(0, 1, 100)))

        # Move the Gaussians to the global origin
        # First get the tangent space rotation
        A = np.eye(model.output_manifold.dim_tangent)
        A[:3, :3] = quaternion_to_rotmat(origin[1][3:])

        # Then move the Gaussians
        for gaussian in gaussians:
            gaussian.tangent_action(A)
            gaussian.action(origin[1])

        # plot the result
        _plot_prob_traj(ax, origin[0], gaussians)

        # plot the origin
        plot_coordinate_system(ax, origin[1], scaling=0.3, labels=None)

    # plot fused result
    _plot_prob_traj(ax, "fused", cast(list[Gaussian], tp_model(np.linspace(0, 1, 100))))

    plt.legend()
    plt.savefig(
        os.path.join(img_dir, f"{name}.png"),
        bbox_inches="tight",
        pad_inches=0,
        dpi=600,
    )
    plt.savefig(
        os.path.join(img_dir, f"{name}.pdf"),
        bbox_inches="tight",
        pad_inches=0,
        dpi=300,
    )
    plt.show()
