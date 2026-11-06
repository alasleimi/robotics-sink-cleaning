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
#include "pglfd/model/gp.h"
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "pglfd/manif/euclidean.h"
#include "pglfd/manif/product.h"
#include "pglfd/model/gaussian.h"
#include "pglfd/model/kernel.h"

using namespace pglfd;
int main() {
	EuclideanManifold3d::Vector identity;
	identity << 0, 0, 0;

	std::vector<typename EuclideanManifold3d::Vector> data_x(3), data_y(3);
	for(int i = 0; i < data_x.size(); i++) {
		data_x[i] << i, i, i;
		data_y[i] << i + 1, i + 1, i + 1;
	}

	std::shared_ptr<RBFKernel<EuclideanManifold3d>> kernel =
			std::make_shared<RBFKernel<EuclideanManifold3d>>(0.001, 1);
	GaussianProcess<EuclideanManifold3d, EuclideanManifold3d> gp =
			GaussianProcess<EuclideanManifold3d, EuclideanManifold3d>(
					data_x,
					data_y,
					std::make_shared<KernelProductIdentity<EuclideanManifold3d, EuclideanManifold3d>>(kernel),
					0.1);

	for(int i = 0; i < data_x.size(); i++) {
		std::cout << (gp({data_x[i]})[0].mu_ - data_y[i]).transpose() << std::endl;
		assert((gp({data_x[i]})[0].mu_ - data_y[i]).norm() < 0.1);
	}

	return 0;
}