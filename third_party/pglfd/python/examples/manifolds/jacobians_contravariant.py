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

import numpy as np
from pglfd.manif.conversions import manifold_from_r3_so3
from pglfd.manif.jacobian import manifold_jacobian
from pglfd.manif.product import ProductManifold

p_r3so3 = np.array([0, 0, 0, 0, 0.0436194, 0, 0.9990482])
vel_r3so3 = np.array([0.1, 0.2, 0.3, 0.01, 0.05, 0.04])

man = ProductManifold.from_string("SO2*R2*SO3")
p_so2r2so3 = manifold_from_r3_so3(man, p_r3so3)
vel_so2r2so3 = manifold_jacobian(man, p_so2r2so3) @ vel_r3so3
vel2_r3so3 = np.linalg.inv(manifold_jacobian(man, p_so2r2so3)) @ vel_so2r2so3
# vel2_r3so3 = vel_r3so3
