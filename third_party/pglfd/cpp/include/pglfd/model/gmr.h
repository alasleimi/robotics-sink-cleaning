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
#include <iostream>
#include <stdexcept>
#include <vector>
#include "pglfd/model/conditioned_gaussian.h"
#include "pglfd/model/gaussian.h"
#include "pglfd/model/gaussian_regressor.h"
#include "pglfd/model/gmm.h"

namespace pglfd {
/**
 * Implementation of Riemannian Gaussian Mixture Regression.
 */
template<typename ManifoldIn, typename ManifoldOut>
class GMR : public GaussianRegressor<ManifoldIn, ManifoldOut> {
public:
	/**
	 * GMR constructor. Please do not call this directly but use `.gmr` of a GMM.
	 *
	 * @param marg_gmm The GMM marginalized to the inputs.
	 * @param cond_gaussians The conditioned Gaussians.
	 * @param marg_gaussians The marginalized Gaussians.
	 */
	GMR(const GMM<ManifoldIn> &marg_gmm,
			const std::vector<ConditionedGaussian<ManifoldIn, ManifoldOut>> &cond_gaussians,
			const std::vector<Gaussian<ManifoldOut>> &marg_gaussians) :
			marg_gmm_(marg_gmm), cond_gaussians_(cond_gaussians), marg_gaussians_(marg_gaussians) {}

	/**
	 * Parameters for GMR regression.
	 */
	struct GmrParameters {
		double eps_marg = 1e-8;    //!< When to use marginalization vs gaussian conditioning.
		double eps_cond = 1e-8;    //!< Stopping criterion for gaussian conditioning.
		double eps_approx = 1e-8;  //!< Stopping criterion for single gaussian approximation.
		int maxit = 50;            //!< Maximum number of iterations to perform before exiting.
	};

	/**
	 * Query the GMR for some output.
	 *
	 * @param x_query The input values.
	 * @param parameters The parameters to be supplied to the method, must be @ref GmrParameters or
	 * empty.
	 * @return The calculated outputs.
	 */
	std::vector<Gaussian<ManifoldOut>> operator()(
			const std::vector<typename ManifoldIn::Vector> &x_query,
			const std::any &parameters = {}) const override {
		GmrParameters params;
		if(parameters.has_value()) {
			params = std::any_cast<const GmrParameters &>(params);
		}
		std::vector<Gaussian<ManifoldOut>> gmr_result;
		auto h = marg_gmm_.expectation(x_query);
		for(int i = 0; i < x_query.size(); i++) {
			std::vector<Gaussian<ManifoldOut>> gauss_selection;
			for(int j = 0; j < h[i].size(); j++) {
				bool use_cond = h[i][j] >= params.eps_marg;
				if(use_cond) {
					try {
						std::vector<typename ManifoldIn::Vector> x_i = {x_query[i]};
						gauss_selection.push_back(cond_gaussians_[j](
								x_i,
								typename ConditionedGaussian<ManifoldIn, ManifoldOut>::
										ConditionedGaussianParameters{params.eps_cond, params.maxit})[0]);
					} catch(const std::runtime_error &err) {
						use_cond = false;
						std::cerr << "Warning: Gaussian condition did not converge, using simple "
												 "marginalization"
											<< std::endl;
					}
				}
				if(!use_cond) {
					gauss_selection.push_back(marg_gaussians_[j]);
				}
			}
			gmr_result.push_back(
					GMM(gauss_selection).approximateSingleGaussian(h[i], params.eps_approx, params.maxit));
		}
		return gmr_result;
	}

private:
	GMM<ManifoldIn> marg_gmm_;
	std::vector<ConditionedGaussian<ManifoldIn, ManifoldOut>> cond_gaussians_;
	std::vector<Gaussian<ManifoldOut>> marg_gaussians_;
};
}  // namespace pglfd