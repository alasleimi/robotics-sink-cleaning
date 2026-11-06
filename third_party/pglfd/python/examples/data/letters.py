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
from typing import Any

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from pglfd.manif.product import ProductManifold
from pglfd.visualization.matplotlib import (
    plot_ellipsoid,
    plot_s2_ellipse,
    plot_s2_tangent,
)

mpl.rcParams["pdf.fonttype"] = 42
mpl.rcParams["ps.fonttype"] = 42


def get_b_s2() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Get the B letter mapped on S2.

    :return: Time and points on S2 and the original trajectories on S2.
    """
    b = np.load(os.path.join(os.path.dirname(__file__), "B.npy"))
    b[:, 0, :] -= np.mean(b[:, 0, :])
    b[:, 1, :] -= np.mean(b[:, 1, :])
    b *= 0.09

    man = ProductManifold.from_string("S2")
    b_s2 = np.zeros((12, 3, 200))
    for i in range(12):
        b_s2[i, :, :] = man.exp(b[i, :, :].T, np.array([0, 0, 1])).T

    t = np.repeat(np.linspace(0, 1, 200), 12)
    y = b_s2.transpose((2, 0, 1)).reshape((-1, 3))

    return t, y, b_s2


def plot_letter_result(
    img_dir: str,
    b_s2: np.ndarray,
    labels: list[str],
    results: list[Any] | None = None,
    *,
    plot_covs: list[Any] | None = None,
    kmeans: list[Any] | None = None,
    gaussians: list[Any] | None = None,
    gmms: list[Any] | None = None,
    samples: list[Any] | None = None,
    vias: list[Any] | None = None,
    lqts: list[Any] | None = None,
) -> None:
    """
    Plot algorithm results for the ``B`` letter.

    :param img_dir: Dir to save data into.
    :param b_s2: The data.
    :param labels: Plot labels.
    :param results: Method results.
    :param plot_covs: Plot covariance of results.
    :param kmeans: KMeans, if any.
    :param gaussians: Gaussians, if any.
    :param gmms: GMM, if any.
    :param samples: Samples from GMM, if any.
    :param vias: Vias, if any.
    :param lqts: LQT, if any.
    """
    # update dir
    img_dir = os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "docs",
        "algorithms",
        "images",
        img_dir,
    )

    # get lqt list
    result_list: list[Any] = results if results is not None else [None for _ in labels]
    plot_cov_list: list[Any] = (
        plot_covs if plot_covs is not None else [True for _ in labels]
    )
    kmeans_list: list[Any] = kmeans if kmeans is not None else [None for _ in labels]
    gaussian_list: list[Any] = (
        gaussians if gaussians is not None else [None for _ in labels]
    )
    gmm_list: list[Any] = gmms if gmms is not None else [None for _ in labels]
    sample_list: list[Any] = samples if samples is not None else [None for _ in labels]
    lqt_list: list[Any] = lqts if lqts is not None else [None for _ in labels]

    # loop through result data
    for name, result, plot_cov, kmean, gaussian, gmm, sample, lqt in zip(
        labels,
        result_list,
        plot_cov_list,
        kmeans_list,
        gaussian_list,
        gmm_list,
        sample_list,
        lqt_list,
        strict=True,
    ):
        # create axis, plot basics
        ax = plt.figure(name).add_subplot(projection="3d")
        plot_ellipsoid(
            ax, np.array([0, 0, 0]), np.diag([1, 1, 1]), color=[0.5, 0.5, 0.5]
        )
        label: str | None = "Demonstrations"
        for i in range(b_s2.shape[0]):
            ax.plot(b_s2[i, 0, :], b_s2[i, 1, :], b_s2[i, 2, :], "y", label=label)
            label = None

        # limits
        ax.set_xlim([-1.0, 1.0])
        ax.set_ylim([-1.0, 1.0])
        ax.set_zlim([-1.0, 1.0])
        ax.set_xticks([-1, -0.5, 0, 0.5, 1])
        ax.set_yticks([-1, -0.5, 0, 0.5, 1])
        ax.set_zticks([-1, -0.5, 0, 0.5, 1])

        # view
        ax.view_init(elev=81.0, azim=-90)

        # pane colors
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor("w")
        ax.yaxis.pane.set_edgecolor("w")
        ax.zaxis.pane.set_edgecolor("w")

        # Plot vias
        if vias is not None:
            label = "Via Points"
            for via in vias:
                ax.plot(via.mu[0], via.mu[1], via.mu[2], "r*", label=label)
                plot_s2_ellipse(
                    ax, via.mu, via.sigma, color=[1, 0, 0, 0.35], use_poly=True
                )
                label = None

        # Plot KMeans
        if kmean is not None:
            label = "Cluster Center"
            cmap = mpl.colormaps["nipy_spectral"]
            colors = cmap(np.linspace(0, 1, len(kmean.cluster_centers)))
            for [center, pts, color] in zip(
                kmean.cluster_centers, kmean.clusters, colors, strict=True
            ):
                ax.plot(pts[:, 1], pts[:, 2], pts[:, 3], ".", color=color, markersize=1)
                ax.plot(center[1], center[2], center[3], "*", color="red", label=label)
                label = None

        # Plot Gaussians
        if gaussian is not None:
            label = "Gaussian"
            offs = 1 if gaussian.mu.shape[0] == 4 else 0
            ax.plot(
                gaussian.mu[offs],
                gaussian.mu[offs + 1],
                gaussian.mu[offs + 2],
                "r*",
                label=label,
            )
            plot_s2_ellipse(
                ax,
                gaussian.mu[offs:],
                gaussian.sigma[offs:, offs:],
                color=[1, 0, 0, 0.35],
                use_poly=True,
            )
            label = None

        # Plot GMM
        if gmm is not None:
            label = "GMM"
            for gauss in gmm.gaussians:
                offs = 1 if gauss.mu.shape[0] == 4 else 0
                ax.plot(
                    gauss.mu[offs],
                    gauss.mu[offs + 1],
                    gauss.mu[offs + 2],
                    "r*",
                    label=label,
                )
                plot_s2_ellipse(
                    ax,
                    gauss.mu[offs:],
                    gauss.sigma[offs:, offs:],
                    color=[1, 0, 0, 0.35],
                    use_poly=True,
                )
                label = None

        # Plot samples
        if sample is not None:
            label = "Samples"
            for pt in sample:
                if pt.shape[0] == 4:
                    ax.plot(pt[1], pt[2], pt[3], "b*", label=label)
                else:
                    ax.plot(pt[0], pt[1], pt[2], "b*", label=label)
                label = None

        # Plot mean
        if result is not None:
            skip = int(len(result) / 100)
            mean = np.array([gauss.mu for gauss in result[::skip]])
            ax.plot(mean[:, 0], mean[:, 1], mean[:, 2], label="Prediction")
            if plot_cov:
                for gauss in result:
                    plot_s2_ellipse(
                        ax, gauss.mu, gauss.sigma, color=[0, 0, 1, 0.1], use_poly=True
                    )

        # Plot velocities for LQT
        if lqt is not None:
            skip = int(len(lqt.poses) / 50)
            label = "velocity"
            for pos, vel, cmd in zip(
                lqt.poses[1:][::skip],
                lqt.velocities[1:][::skip],
                lqt.commands[::skip],
                strict=True,
            ):
                plot_s2_tangent(ax, pos, vel * 0.04, "b", None, 1, 3)
                plot_s2_tangent(ax, pos, cmd * lqt.dt * 0.04, "g", None, 1, 3)

        plt.legend()
        plt.savefig(
            os.path.join(img_dir, f"{name}.png"),
            bbox_inches="tight",
            pad_inches=0,
            dpi=300,
        )
        plt.savefig(
            os.path.join(img_dir, f"{name}.pdf"),
            bbox_inches="tight",
            pad_inches=0,
            dpi=300,
        )

    plt.show()
