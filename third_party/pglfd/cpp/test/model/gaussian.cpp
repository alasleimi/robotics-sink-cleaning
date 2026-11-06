/*
 * Copyright (c) 2026 Maximilian Mühlbauer
 *
 * This file is part of pglfd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif
#include "pglfd/model/gaussian.h"
#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <string>
#include <vector>
#include "pglfd/manif/product.h"

using namespace pglfd;
int main() {
	Eigen::Vector4d unit_quat;
	unit_quat << 0, 0, 0, 1;
	typename SE3Manifoldd::Scalar reg = 1e-3;

	std::vector<typename SE3Manifoldd::Vector> data(100);
	std::vector<typename SE3Manifoldd::Scalar> weights(100);
	for(int i = 0; i < 100; i++) {
		data[i].block<SE3Manifoldd::dim_manifold - 4, 1>(0, 0) =
				SE3Manifoldd::Vector::Random().block<SE3Manifoldd::dim_manifold - 4, 1>(0, 0);
		data[i].block<4, 1>(SE3Manifoldd::dim_manifold - 4, 0) = unit_quat;
		weights[i] = 1;
	}
	Gaussian<SE3Manifoldd> gauss = Gaussian<SE3Manifoldd>::mleFromData(data, weights, reg);
	assert((gauss.mu_.block<4, 1>(SE3Manifoldd::dim_manifold - 4, 0) - unit_quat)
						 .isMuchSmallerThan(1e-10));
	assert((gauss.sigma_.block<3, 3>(SE3Manifoldd::dim_tangent - 3, SE3Manifoldd::dim_tangent - 3)
					- Eigen::Matrix3d::Identity() * reg)
						 .isMuchSmallerThan(1e-10));

	Gaussian<SE3Manifoldd> product = (gauss * gauss * gauss * (gauss * gauss));
	assert((product.mu_ - gauss.mu_).isMuchSmallerThan(1e-5));
	double fac = 1. / 5.;  // We "decrease" the uncertainty by factor 5 when multiplying 5 times
	assert((product.sigma_ - fac * gauss.sigma_).isMuchSmallerThan(1e-2));

	return 0;
}