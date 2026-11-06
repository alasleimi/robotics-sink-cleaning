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
#include <memory>
#include <stdexcept>
#include <vector>
#include "pglfd/model/gaussian.h"
#include "pglfd/model/gaussian_regressor.h"
#include "pglfd/model/kernel.h"

namespace pglfd {
/**
 * Riemannian Nadaraya-Watson regression implementation.
 *
 * @param ManifoldIn The input data manifold.
 * @param ManifoldOut The output data manifold.
 */
template<typename ManifoldIn, typename ManifoldOut>
class NadarayaWatson : public GaussianRegressor<ManifoldIn, ManifoldOut> {
public:
	/**
	 * Constructor.
	 *
	 * @param x Input values.
	 * @param y Output values.
	 * @param kernel The kernel to compute distances between query values and data points.
	 */
	NadarayaWatson(
			const std::vector<typename ManifoldIn::Vector> &x,
			const std::vector<typename ManifoldOut::Vector> &y,
			const std::shared_ptr<Kernel<ManifoldIn>> &kernel) :
			x_(x), y_(y), kernel_(kernel) {
		if(x.size() != y.size()) {
			throw std::runtime_error("sizes of input data output data must agree!");
		}
	}
	virtual ~NadarayaWatson() {}

	/**
	 * Parameters for Nadaraya-Watson regression.
	 */
	struct NadarayaWatsonParameters {
		double reg = 1e-8;  //!< Regularization constant for the covariance estimation.
		double eps = 1e-8;  //!< Epsilon value for the frechet mean.
		int maxit = 50;     //!< Maximum number of iterations to perform before exiting.
	};

	/**
	 * Query the Nadaraya-Watson regressor for some output.
	 *
	 * @param x_query The input values.
	 * @param parameters The parameters to be supplied to the method, must be @ref
	 * NadarayaWatsonParameters or empty.
	 * @return The calculated outputs.
	 */
	std::vector<Gaussian<ManifoldOut>> operator()(
			const std::vector<typename ManifoldIn::Vector> &x_query,
			const std::any &parameters = {}) const override {
		NadarayaWatsonParameters params;
		if(parameters.has_value()) {
			params = std::any_cast<const NadarayaWatsonParameters &>(params);
		}
		// Preallocate result
		std::vector<Gaussian<ManifoldOut>> res;
		for(const auto &x_i: x_query) {
			res.emplace_back(manif_out_.id_elem(), ManifoldOut::TangentMatrix::Zero());
		}

		// Multi-threaded calculation
#pragma omp parallel for
		for(int i = 0; i < x_query.size(); i++) {
			// Calculate weights
			std::vector<typename ManifoldIn::Scalar> weights(x_.size());
			for(int j = 0; j < x_.size(); j++) {
				weights[j] = (*kernel_)(x_[j], x_query[i]);
			}
			res[i] =
					Gaussian<ManifoldOut>::mleFromData(y_, weights, params.reg, params.eps, params.maxit);
		}
		return res;
	}

protected:
	// Manifold stuff
	ManifoldIn manif_in_;    //!< The input data manifold.
	ManifoldOut manif_out_;  //!< The output data manifold.

	// Parameters & Data
	std::vector<typename ManifoldIn::Vector> x_;   //!< The input data.
	std::vector<typename ManifoldOut::Vector> y_;  //!< The output data.
	std::shared_ptr<Kernel<ManifoldIn>> kernel_;   //!< The kernel.
};
}  // namespace pglfd
