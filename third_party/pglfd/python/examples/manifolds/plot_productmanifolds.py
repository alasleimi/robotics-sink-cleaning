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

import matplotlib.pyplot as plt
import numpy as np
from pglfd.manif.conversions import manifold_to_r3_so3
from pglfd.manif.product import ProductManifold
from pglfd.visualization.matplotlib import plot_coordinate_system

img_dir = os.path.join(
    os.path.dirname(__file__),
    "..",
    "..",
    "..",
    "docs",
    "algorithms",
    "images",
    "manifold",
)

# TODO make this code nice / remove stuff we don't need

# Create axis
axes = [5, 5, 5]

# Create Data
data = np.ones(axes, dtype=bool)

# Control Transparency
alpha = 0.5
color = [21 / 255, 96 / 255, 130 / 255, alpha]

# Control colour
colors = np.empty([*axes, 4], dtype=np.float32)
colors[:] = color

# Plot figure
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Voxels is used to customizations of the
# sizes, positions and colors.
ax.voxels(data, facecolors=colors)
ax.set_xticks([])
ax.set_yticks([])
ax.set_zticks([])
ax.axis("off")
plot_coordinate_system(ax, np.array([2.5, 2.5, 2.5, 0, 0, 0, 1]), scaling=1)
plot_coordinate_system(ax, np.array([0, 2.5, 2.5, 0, 0, 0, 1]), scaling=1)
plot_coordinate_system(ax, np.array([2.5, 0, 2.5, 0, 0, 0, 1]), scaling=1)
plot_coordinate_system(ax, np.array([2.5, 2.5, 0, 0, 0, 0, 1]), scaling=1)
plot_coordinate_system(ax, np.array([5, 2.5, 2.5, 0, 0, 0, 1]), scaling=1)
plot_coordinate_system(ax, np.array([2.5, 5, 2.5, 0, 0, 0, 1]), scaling=1)
plot_coordinate_system(ax, np.array([2.5, 2.5, 5, 0, 0, 0, 1]), scaling=1)

plt.savefig(
    os.path.join(img_dir, "cartesian.png"),
    bbox_inches="tight",
    pad_inches=-0.15,
    dpi=300,
)


#
# Cylinder
#
def _data_for_cylinder_along_z(
    center_x: float, center_y: float, radius: float, height_z: float
) -> tuple:
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2 * np.pi, 50)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x_grid = radius * np.cos(theta_grid) + center_x
    y_grid = radius * np.sin(theta_grid) + center_y
    return x_grid, y_grid, z_grid


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

Xc, Yc, Zc = _data_for_cylinder_along_z(0, 0, 5, 5)
ax.plot_surface(Xc, Yc, Zc, alpha=alpha)

ax.set_xticks([])
ax.set_yticks([])
ax.set_zticks([])
ax.axis("off")
plot_coordinate_system(ax, np.array([0, 0, 0, 0, 0, 0, 1]), scaling=1)
man = ProductManifold.from_string("SO2*R2*SO3")
plot_coordinate_system(
    ax, manifold_to_r3_so3(man, np.array([1, 0, 5, 0, 0, 0, 0, 1])), scaling=1
)
plot_coordinate_system(
    ax, manifold_to_r3_so3(man, np.array([0, 1, 5, 0, 0, 0, 0, 1])), scaling=1
)
plot_coordinate_system(
    ax, manifold_to_r3_so3(man, np.array([-1, 0, 5, 0, 0, 0, 0, 1])), scaling=1
)
plot_coordinate_system(
    ax, manifold_to_r3_so3(man, np.array([0, -1, 5, 0, 0, 0, 0, 1])), scaling=1
)

plt.savefig(
    os.path.join(img_dir, "cylindrical.png"),
    bbox_inches="tight",
    pad_inches=-0.4,
    dpi=300,
)


#
# Sphere
#
def _plt_sphere(
    ax: plt.Axes, list_center: list, list_radius: list, color: list[float], alpha: float
) -> None:
    # TODO replace with function from visualization?!?
    for c, r in zip(list_center, list_radius, strict=True):
        # draw sphere
        u, v = np.mgrid[0 : 2 * np.pi : 50j, 0 : np.pi : 50j]  # type: ignore[misc]
        x = r * np.cos(u) * np.sin(v)
        y = r * np.sin(u) * np.sin(v)
        z = r * np.cos(v)

        ax.plot_surface(x - c[0], y - c[1], z - c[2], color=color, alpha=alpha)


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

_plt_sphere(ax, [(0, 0, 0)], [5], color, alpha)

ax.set_xticks([])
ax.set_yticks([])
ax.set_zticks([])
ax.axis("off")
plot_coordinate_system(ax, np.array([0, 0, 0, 0, 0, 0, 1]), scaling=1)
man = ProductManifold.from_string("S2*R1*SO3")
plot_coordinate_system(
    ax, manifold_to_r3_so3(man, np.array([1, 0, 0, 5, 0, 0, 0, 1])), scaling=1
)
plot_coordinate_system(
    ax, manifold_to_r3_so3(man, np.array([0, 1, 0, 5, 0, 0, 0, 1])), scaling=1
)
plot_coordinate_system(
    ax, manifold_to_r3_so3(man, np.array([-1, 0, 0, 5, 0, 0, 0, 1])), scaling=1
)
plot_coordinate_system(
    ax, manifold_to_r3_so3(man, np.array([0, -1, 0, 5, 0, 0, 0, 1])), scaling=1
)

plt.savefig(
    os.path.join(img_dir, "spherical.png"),
    bbox_inches="tight",
    pad_inches=-0.5,
    dpi=300,
)

plt.show()
