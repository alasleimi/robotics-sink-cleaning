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
#include "pglfd/model/gaussian.h"
#include "pglfd/model/gaussian_regressor.h"

namespace pglfd {
/**
 * Implementation of Riemannian Gaussian conditioning.
 */
template<typename ManifoldIn, typename ManifoldOut>
class ConditionedGaussian : public GaussianRegressor<ManifoldIn, ManifoldOut> {
public:
	/**
	 * Constructor for the conditioned Gaussian. Please call ``.condition`` on a Gaussian instead of
	 * directly using this constructor.
	 *
	 * @param gauss_marg_in The Gaussian marginalized to the input.
	 * @param gauss_marg_out The Gaussian marginalized to the output.
	 * @param sigma_oi Correlation matrix between input and output.
	 */
	ConditionedGaussian(
			const Gaussian<ManifoldIn> &gauss_marg_in,
			const Gaussian<ManifoldOut> &gauss_marg_out,
			const Eigen::
					Matrix<typename ManifoldIn::Scalar, ManifoldOut::dim_tangent, ManifoldIn::dim_tangent>
							&sigma_oi) :
			gauss_marg_in_(gauss_marg_in),
			gauss_marg_out_(gauss_marg_out),
			sigma_oi_(sigma_oi),
			sigma_ii_inv_(gauss_marg_in.sigma_.inverse()) {}

	/**
	 * Parameters for conditioned Gaussian regression.
	 */
	struct ConditionedGaussianParameters {
		double eps = 1e-8;  //!< Threshold parameter.
		int maxit = 50;     //!< Maximum number of iterations.
	};

	/**
	 * Calculate the conditioned Gaussian for input x. Please see the Python documentation for
	 * details.
	 *
	 * @param x The input vectors.
	 * @param parameters The parameters to be supplied to the method, must be @ref
	 * ConditionedGaussianParameters or empty.
	 * @return Gaussians resulting from the conditioning operation.
	 */
	std::vector<Gaussian<ManifoldOut>> operator()(
			const std::vector<typename ManifoldIn::Vector> &x,
			const std::any &parameters = {}) const override {
		ConditionedGaussianParameters params;
		if(parameters.has_value()) {
			params = std::any_cast<const ConditionedGaussianParameters &>(params);
		}
		std::vector<Gaussian<ManifoldOut>> res;
		for(auto &x_i: x) {
			typename ManifoldOut::Vector mu = gauss_marg_out_.mu_;
			typename ManifoldIn::TangentVector log_x_in =
					gauss_marg_in_.manif_.log(x_i, gauss_marg_in_.mu_);

			// We want to keep some data after the loop
			typename ManifoldOut::TangentMatrix R_out;

			// Loop
			int it = 0;
			double diff = std::numeric_limits<double>::max();
			while(diff > params.eps) {
				// Get parallel transport
				R_out = gauss_marg_out_.manif_.parallelTransportTrafo(mu, gauss_marg_out_.mu_);

				// Calculate update
				typename ManifoldOut::TangentVector delta =
						-R_out.transpose() * gauss_marg_out_.manif_.log(mu, gauss_marg_out_.mu_)
						+ R_out.transpose() * sigma_oi_ * sigma_ii_inv_ * log_x_in;
				mu = gauss_marg_out_.manif_.exp(delta, mu);

				// Check for convergence
				diff = delta.transpose() * delta;
				it++;
				if(it > params.maxit) {
					throw std::runtime_error("Gaussian conditioning did not converge");
				}
			}

			// Calculate sigma
			R_out = gauss_marg_out_.manif_.parallelTransportTrafo(mu, gauss_marg_out_.mu_);
			typename ManifoldOut::TangentMatrix sigma =
					R_out.transpose()
					* (gauss_marg_out_.sigma_ - sigma_oi_ * sigma_ii_inv_ * sigma_oi_.transpose()) * R_out;

			// Append
			res.emplace_back(mu, sigma);
		}
		return res;
	}

private:
	Gaussian<ManifoldIn> gauss_marg_in_;
	Gaussian<ManifoldOut> gauss_marg_out_;
	Eigen::Matrix<typename ManifoldIn::Scalar, ManifoldOut::dim_tangent, ManifoldIn::dim_tangent>
			sigma_oi_;
	typename ManifoldIn::TangentMatrix sigma_ii_inv_;
};
}  // namespace pglfd
