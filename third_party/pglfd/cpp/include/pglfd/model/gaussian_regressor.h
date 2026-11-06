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

#pragma once
#include <any>
#include <vector>
#include "pglfd/manif/conversions.h"
#include "pglfd/manif/jacobian.h"
#include "pglfd/manif/product.h"
#include "pglfd/model/gaussian.h"

namespace pglfd {
template<typename ManifoldIn, typename ScalarOut>
class GaussianRegressorR3SO3 {
public:
	/**
	 * Query the Gaussian regressor for some output in R3*SO(3).
	 *
	 * @param x_query The input values.
	 * @param parameters The parameters to be supplied to the method.
	 * @return The calculated outputs.
	 */
	virtual std::vector<Gaussian<SE3Manifold<ScalarOut>>> query_r3_so3(
			const std::vector<typename ManifoldIn::Vector> &x_query,
			const std::any &parameters = {}) const = 0;
	virtual ~GaussianRegressorR3SO3() = default;
};

template<typename ManifoldIn, typename ManifoldOut>
class GaussianRegressor : public GaussianRegressorR3SO3<ManifoldIn, typename ManifoldOut::Scalar> {
public:
	/**
	 * Query the Gaussian regressor for some output.
	 *
	 * @param x_query The input values.
	 * @param parameters The parameters to be supplied to the method.
	 * @return The calculated outputs.
	 */
	virtual std::vector<Gaussian<ManifoldOut>> operator()(
			const std::vector<typename ManifoldIn::Vector> &x_query,
			const std::any &parameters = {}) const = 0;

	std::vector<Gaussian<SE3Manifold<typename ManifoldOut::Scalar>>> query_r3_so3(
			const std::vector<typename ManifoldIn::Vector> &x_query,
			const std::any &parameters = {}) const override {
		// Query on-manifold result
		std::vector<Gaussian<ManifoldOut>> res_manif = this->operator()(x_query, parameters);

		if constexpr(std::is_same_v<ManifoldOut, SE3Manifold<typename ManifoldOut::Scalar>>) {
			return res_manif;
		} else if constexpr(
				std::is_same_v<ManifoldOut, CylindricalManifold<typename ManifoldOut::Scalar>>
				|| std::is_same_v<ManifoldOut, SphericalManifold<typename ManifoldOut::Scalar>>) {
			// Transform to R3*SO3
			std::vector<Gaussian<SE3Manifold<typename ManifoldOut::Scalar>>> res_r3_so3;
			res_r3_so3.reserve(res_manif.size());
			for(const auto &res: res_manif) {
				typename SE3Manifold<typename ManifoldOut::Scalar>::TangentMatrix jacobian =
						manifold_jacobian<ManifoldOut>(res.mu_).inverse();
				typename SE3Manifold<typename ManifoldOut::Scalar>::TangentMatrix sigma_transformed =
						jacobian * res.sigma_ * jacobian.transpose();
				res_r3_so3.emplace_back(manifold_to_r3_so3<ManifoldOut>(res.mu_), sigma_transformed);
			}
			return res_r3_so3;
		}
		throw std::runtime_error("Manifold cannot be converted to R3*SO(3)!");
	}
};
}  // namespace pglfd