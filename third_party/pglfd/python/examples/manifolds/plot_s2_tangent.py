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
from pglfd.manif.product import ProductManifold
from pglfd.visualization.matplotlib import (
    plot_coordinate_system,
    plot_ellipsoid,
    plot_s2_tangent,
)

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

# Control Transparency
alpha = 0.5
color = [245 / 255, 149 / 255, 66 / 255, alpha]

#
# Sphere
#
man_s2 = ProductManifold.from_string("S2")

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


plot_ellipsoid(ax, np.array([0, 0, 0]), np.diag([1, 1, 1]), color=[0.5, 0.5, 0.5])

ax.set_xlim([-1.0, 1.0])
ax.set_ylim([-1.0, 1.0])
ax.set_zlim([-1.0, 1.0])

ax.set_xticks([])
ax.set_yticks([])
ax.set_zticks([])
ax.axis("off")
plot_coordinate_system(ax, np.array([0, 0, 1, 0, 0, 0, 1]), scaling=1)

# add tangent space
off = 0.70710678118
x = np.linspace(-4 / 5, 4 / 5, 100) + off
y = np.linspace(-2 / 5, 2 / 5, 100)

x, y = np.meshgrid(x, y)
eq = -(x - off) + (0 * y + 0) + off
ax.plot_surface(x, y, eq)

# points
plt.plot(off, 0, off, "ko")
plt.plot(-off, 0, -off, "ko")
u = np.array([0.5, 0])
plot_s2_tangent(ax, np.array([off, 0, off]), u, color="y")
plt.plot(u[0] * 0.70710678118 + off, 0, -u[0] * 0.70710678118 + off, "ko")
pt = man_s2.exp(u, np.array([off, 0, off]))
plt.plot(pt[0], pt[1], pt[2], "ko")

# TODO make code nice, plot text as on slide 15
# https://matplotlib.org/stable/gallery/mplot3d/text3d.html

plt.savefig(
    os.path.join(img_dir, "s2_tangent.png"),
    bbox_inches="tight",
    pad_inches=0,
    dpi=300,
)
plt.savefig(
    os.path.join(img_dir, "s2_tangent.pdf"),
    bbox_inches="tight",
    pad_inches=0,
    dpi=300,
)

plt.show()
