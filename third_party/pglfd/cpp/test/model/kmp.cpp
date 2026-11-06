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
#include "pglfd/model/kmp.h"
#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "pglfd/manif/euclidean.h"
#include "pglfd/manif/product.h"
#include "pglfd/manif/so3.h"
#include "pglfd/model/gaussian.h"
#include "pglfd/model/gmm.h"
#include "pglfd/model/gmr.h"
#include "pglfd/model/kernel.h"

#define N_POINTS 10

using namespace pglfd;
int main() {
	using Manif = ProductManifold<double, EuclideanManifold1d, EuclideanManifold3d, SO3Manifoldd>;
	using ManifIn = ProductManifold<double, EuclideanManifold1d>;
	using ManifOut = ProductManifold<double, EuclideanManifold3d, SO3Manifoldd>;
	Eigen::Vector4d unit_quat;
	unit_quat << 0, 0, 0, 1;
	typename Manif::Scalar reg = 1e-3;

	std::vector<Gaussian<Manif>> gaussians;
	int offset = 0;
	for(int j = 0; j < 10; j++) {
		std::vector<typename Manif::Vector> data(N_POINTS);
		std::vector<typename Manif::Scalar> weights(N_POINTS);
		for(int i = 0; i < N_POINTS; i++) {
			data[i][0] = i + offset;
			data[i][1] = i * 0.05 + offset * 0.1;
			data[i][2] = i * 0.05 + offset * 0.1;
			data[i][3] = i * 0.05 + offset * 0.1;
			data[i].block<4, 1>(4, 0) = unit_quat;
			weights[i] = 1;
		}
		gaussians.push_back(Gaussian<Manif>::mleFromData(data, weights, reg));
		offset += N_POINTS * 2;
	}

	GMM<Manif> gmm = GMM<Manif>(gaussians);
	GMR<ManifIn, ManifOut> gmr = gmm.gmr(manifold_selector<0>{}, manifold_selector<1, 2>{});

	std::vector<typename ManifIn::Vector> data_in;
	for(int i = 0; i < N_POINTS; i++) {
		data_in.push_back(ManifIn::Vector(i * 2 * N_POINTS + N_POINTS / 2));
	}
	std::vector<Gaussian<ManifOut>> gaussian_out = gmr(data_in);
	std::vector<typename ManifOut::Vector> data_out;
	std::vector<typename ManifOut::TangentMatrix> covariance_out;
	for(auto& gaussian: gaussian_out) {
		data_out.push_back(gaussian.mu_);
		covariance_out.push_back(gaussian.sigma_);
	}

	std::shared_ptr<KernelProductIdentity<ManifIn, ManifOut>> kernel =
			std::make_shared<KernelProductIdentity<ManifIn, ManifOut>>(
					std::make_shared<RBFKernel<ManifIn>>(0.001, 1));
	KMP<ManifIn, ManifOut> kmp = KMP<ManifIn, ManifOut>(data_in, data_out, kernel, covariance_out);

	for(int i = 0; i < data_in.size(); i++) {
		std::cout << data_in[i] << std::endl;
		std::cout << kmp({data_in[i]})[0].mu_.transpose() << std::endl;
		std::cout << data_out[i].transpose() << std::endl;
		std::cout << (kmp({data_in[i]})[0].mu_ - data_out[i]).transpose() << std::endl;
		assert((kmp({data_in[i]})[0].mu_ - data_out[i]).norm() < 0.1);
	}

	// append measurments
	ManifIn::Vector x_new(100);
	ManifOut::Vector y_new(1, 2, 3, 0, 0, 0, 1);
	kmp.appendMeasurements({x_new}, {y_new}, {ManifOut::TangentMatrix::Identity() * 1e-3});

	return 0;
}
