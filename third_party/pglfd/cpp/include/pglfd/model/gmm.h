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
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <iostream>
#include <random>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>
#include "pglfd/manif/product.h"
#include "pglfd/manif/util.h"
#include "pglfd/model/conditioned_gaussian.h"
#include "pglfd/model/gaussian.h"
#include "pglfd/model/kmeans.h"
#include "pglfd/util/execution.h"

namespace pglfd {
template<typename ManifoldIn, typename ManifoldOut>
class GMR;  // IWYU pragma: keep

/**
 * Implementation of a Riemannian Gaussian Mixture Model.
 */
template<typename Manifold>
class GMM {
public:
	/**
	 * Constructor of the GMM.
	 *
	 * @param gaussians The gaussians this GMM consists of.
	 * @param priors The priors for the gaussians, can also be left empty.
	 */
	explicit GMM(
			const std::vector<Gaussian<Manifold>> &gaussians, const std::vector<double> &priors = {}) :
			gaussians_(gaussians), priors_(priors) {
		if(priors_.size() != gaussians_.size()) {
			priors_.clear();
			for(int i = 0; i < gaussians_.size(); i++) {
				priors_.push_back(1. / gaussians_.size());
			}
		}
	}

	/**
	 * Rotate our tangent space by A (pure rotation of the tangent space at the origin, no translation
	 * on the manifold).
	 *
	 * @param A Rotation to perform in tangent space.
	 */
	void tangentAction(const typename Manifold::TangentMatrix &A) {
		for(auto &gaussian: gaussians_) {
			gaussian.tangentAction(A);
		}
	}

	/**
	 * Perform the action function on the Gaussian (pure translation, no rotation of the tangent
	 * space).
	 *
	 * @param trafo Transformation to apply.
	 */
	void action(const typename Manifold::Vector &trafo) {
		for(auto &gaussian: gaussians_) {
			gaussian.action(trafo);
		}
	}

	/**
	 * Calculate the expectation for data points.
	 *
	 * @param data The datapoints.
	 * @param recompute_decomposition Force to recompute the decomposition of sigma_.
	 * @return The resulting likelihood for each of the GMM's gaussians
	 */
	std::vector<std::vector<typename Manifold::Scalar>> expectation(
			const std::vector<typename Manifold::Vector> &data,
			bool recompute_decomposition = true) const {
		// First calculate the shifted result: gaussians outside, datapoints inside
		std::vector<std::vector<typename Manifold::Scalar>> res_shifted(gaussians_.size());
#pragma omp parallel for
		for(int i = 0; i < gaussians_.size(); i++) {
			res_shifted[i] = gaussians_[i].probability(data, recompute_decomposition);
		}

		// Reorder to the result we want: datapoints outside, gaussians inside
		// additionally incorporate priors
		std::vector<std::vector<typename Manifold::Scalar>> res(res_shifted[0].size());
#pragma omp parallel for
		for(int i = 0; i < res_shifted[0].size(); i++) {
			for(int j = 0; j < res_shifted.size(); j++) {
				res[i].push_back(res_shifted[j][i] * priors_[j]);
			}
		}
		return res;
	}

	/**
	 * Sample from this GMM.
	 * See https://stats.stackexchange.com/a/419399 and numpy code.
	 *
	 * @param n The number of samples to draw.
	 * @return Samples.
	 */
	std::vector<typename Manifold::Vector> sample(int n) {
		// Initialize the RNG
		static std::mt19937 gen{std::random_device{}()};

		// Initialize priors using binomial sampling
		std::vector<int> pis;
		pis.resize(gaussians_.size());
		typename Manifold::Scalar remaining_p = 1.0;
		int remaining_n = n;
		for(int i = 0; i < gaussians_.size() - 1; i++) {
			std::binomial_distribution<> binomial(remaining_n, priors_[i] / remaining_p);
			pis[i] = binomial(gen);
			remaining_p -= priors_[i];
			remaining_n -= pis[i];
		}
		pis[gaussians_.size() - 1] = remaining_n;

		// Sample
		std::vector<typename Manifold::Vector> res;
		for(int i = 0; i < gaussians_.size(); i++) {
			std::vector<typename Manifold::Vector> resi = gaussians_[i].sample(pis[i]);
			res.insert(res.end(), resi.begin(), resi.end());
		}
		return res;
	}

	/**
	 * Calculate the marginal distribution.
	 *
	 * @param idx_man_val Manifold dimensions.
	 * @return The marginal distribution.
	 */
	template<std::ptrdiff_t... idx_man>
	auto margin(const manifold_selector<idx_man...> &idx_man_val) const {
		auto man = manif_.template submanifold<idx_man...>();
		std::vector<Gaussian<std::tuple_element_t<0, decltype(man)>>> marg_gaussians;
		marg_gaussians.reserve(gaussians_.size());
		std::transform(
				gaussians_.begin(),
				gaussians_.end(),
				std::back_inserter(marg_gaussians),
				[&](const auto &gaussian) -> Gaussian<std::tuple_element_t<0, decltype(man)>> {
					return gaussian.margin(idx_man_val);
				});
		return GMM<std::tuple_element_t<0, decltype(man)>>(marg_gaussians, priors_);
	}

	/**
	 * Gaussian mixture regression. Returns a GMR which can be called with
	 * the concrete input values.
	 *
	 * @param idx_in_val Input manifolds.
	 * @param idx_out_val Output manifolds.
	 * @return The GMR.
	 */
	template<std::ptrdiff_t... idx_in, std::ptrdiff_t... idx_out>
	auto gmr(
			const manifold_selector<idx_in...> &idx_in_val,
			const manifold_selector<idx_out...> &idx_out_val) const {
		auto man_tot = manif_.template submanifold<idx_in..., idx_out...>();
		auto man_in = manif_.template submanifold<idx_in...>();
		auto man_out = manif_.template submanifold<idx_out...>();

		// Get marginalized GMM & Gaussians
		auto marg_gmm = margin(idx_in_val);
		std::vector<ConditionedGaussian<
				std::tuple_element_t<0, decltype(man_in)>,
				std::tuple_element_t<0, decltype(man_out)>>>
				cond_gaussians;
		std::vector<Gaussian<std::tuple_element_t<0, decltype(man_out)>>> marg_gaussians;

		for(const auto &gaussian: gaussians_) {
			cond_gaussians.push_back(gaussian.condition(idx_in_val, idx_out_val));
			marg_gaussians.push_back(gaussian.margin(idx_out_val));
		}

		return GMR<
				std::tuple_element_t<0, decltype(man_in)>,
				std::tuple_element_t<0, decltype(man_out)>>(marg_gmm, cond_gaussians, marg_gaussians);
	}

	/**
	 * Approximate the GMM by a single gaussian.
	 *
	 * @param weights Weights for the individual gaussian components.
	 * @param eps Stopping criterion.
	 * @param maxit Maximum number of iterations.
	 * @return The approximated gaussian.
	 */
	Gaussian<Manifold> approximateSingleGaussian(
			const std::vector<typename Manifold::Scalar> &weights, double eps = 1e-8, int maxit = 50) {
		assert(weights.size() == gaussians_.size());

		// Prepare weights
		std::vector<typename Manifold::Scalar> w = weights;
		typename Manifold::Scalar weight_sum = 0;
		weight_sum = std::accumulate(w.begin(), w.end(), weight_sum);
		for(typename Manifold::Scalar &weight: w) {
			if(std::isfinite(weight_sum) && weight_sum > 0)
				weight /= weight_sum;
			else
				weight = 1. / (double)w.size();
		}

		// Extract means
		std::vector<typename Manifold::Vector> mus(gaussians_.size());
		std::transform(
				execution::unseq,
				gaussians_.begin(),
				gaussians_.end(),
				mus.begin(),
				[](const auto &gaussian) { return gaussian.mu_; });

		// Mean is the weighted average
		typename Manifold::Vector mu = frechet_mean<Manifold>(mus, w, eps, maxit);

		// Covariance is the weighted average + moments
		typename Manifold::TangentMatrix sigma;
		sigma.setZero();
		for(int i = 0; i < gaussians_.size(); i++) {
			typename Manifold::TangentVector logval = manif_.log(gaussians_[i].mu_, mu);
			typename Manifold::TangentMatrix moment = logval * logval.transpose();
			typename Manifold::TangentMatrix sigma_transp =
					manif_.parallelTransportMatrix(gaussians_[i].sigma_, gaussians_[i].mu_, mu);
			sigma += w[i] * (sigma_transp + moment);
		}

		return Gaussian<Manifold>(mu, sigma);
	}

	/**
	 * Initialization algorithm for a GMM.
	 */
	enum InitAlgorithm {
		KMEANS  //!< Initialization using the KMeans algorithm.
	};

	/**
	 * Fit the GMM using some initially fitted gaussians (e.g. using k-means) and some data.
	 *
	 * @param k The number of components to fit.
	 * @param data The data to fit.
	 * @param init_algorithm The initialization algorithm to use. Currently only "kmeans".
	 * @param eps Stopping criterion.
	 * @param maxit Maximum number of iterations to perform before exiting.
	 * @return tuple The fitted GMM, likelihood values for the data, list of average likelihoods
	 *during optimization.
	 **/
	static std::tuple<
			GMM<Manifold>,
			std::vector<std::vector<typename Manifold::Scalar>>,
			std::vector<typename Manifold::Scalar>>
			fit(int k,
					const std::vector<typename Manifold::Vector> &data,
					InitAlgorithm init_algorithm,
					typename Manifold::Scalar eps = 1e-5,
					int maxit = 100) {
		std::vector<typename Manifold::Scalar> priors;
		std::vector<Gaussian<Manifold>> gaussians;

		// Initial clustering
		switch(init_algorithm) {
			case KMEANS: {
				KMeans<Manifold> kmeans = KMeans<Manifold>::fit(k, data);
				for(int i = 0; i < k; i++) {
					priors.push_back(
							((typename Manifold::Scalar)kmeans.clusters_[i].size())
							/ ((typename Manifold::Scalar)data.size()));
					gaussians.push_back(Gaussian<Manifold>::mleFromData(kmeans.clusters_[i]));
				}
			} break;
			default:
				throw std::runtime_error("Unknown GMM initialization algorithm");
		}

		// Initialize GMM
		GMM<Manifold> gmm(gaussians, priors);

		// Optimization loop
		std::vector<std::vector<typename Manifold::Scalar>> likelihood;
		std::vector<typename Manifold::Scalar> avg_loglikelihood;
		int it = 0;
		typename Manifold::Scalar cur_likelihood = 2 * eps, prev_likelihood = 0;
		while(cur_likelihood - prev_likelihood > eps) {
			// Storage
			prev_likelihood = cur_likelihood;
			// Expectation Step
			// Variable names after https://www.ics.uci.edu/~smyth/courses/cs274/notes/EMnotes.pdf
			likelihood = gmm.expectation(data, false);

			// Maximization Step
			// - update gaussian
			std::vector<typename Manifold::Scalar> lik_sums(likelihood.size());
			std::transform(
					execution::unseq,
					likelihood.begin(),
					likelihood.end(),
					lik_sums.begin(),
					[](const auto &lik_row) {
						typename Manifold::Scalar lik_sum = 0;
						return std::reduce(execution::unseq, lik_row.begin(), lik_row.end(), lik_sum);
					});

			std::vector<std::vector<typename Manifold::Scalar>> weights;
			weights.resize(k);
#pragma omp parallel for
			for(int i = 0; i < k; i++) {
				for(int j = 0; j < data.size(); j++) {
					const std::vector<typename Manifold::Scalar> &lik_row = likelihood[j];
					weights[i].push_back(lik_row[i] / lik_sums[j]);
				}
				gmm.gaussians_[i] = Gaussian<Manifold>::mleFromData(data, weights[i]);
			}

			// - update priors
			typename Manifold::Scalar prior_sum = 0;
			for(int i = 0; i < k; i++) {
				gmm.priors_[i] = 0;
				for(int j = 0; j < data.size(); j++) {
					gmm.priors_[i] += weights[i][j];
				}
				prior_sum += gmm.priors_[i];
			}
			for(int i = 0; i < k; i++) {
				gmm.priors_[i] /= prior_sum;
			}

			// Convergence check
			cur_likelihood = 0;
			cur_likelihood = std::transform_reduce(
					execution::unseq,
					lik_sums.begin(),
					lik_sums.end(),
					cur_likelihood,
					std::plus{},
					[](const auto &lik_sum) { return std::log(lik_sum + 1e-200); });
			cur_likelihood /= lik_sums.size();
			avg_loglikelihood.push_back(cur_likelihood);

			// Maximum number of iterations
			it++;
			if(it > maxit) {
				std::cerr << "Likelihood over iterations: ";
				for(const auto &elem: avg_loglikelihood) {
					std::cerr << elem << ", ";
				}
				std::cerr << std::endl;
				throw std::runtime_error("GMM Expectation Maximization did not converge.");
			}
		}

		return {gmm, likelihood, avg_loglikelihood};
	}

	std::vector<Gaussian<Manifold>> gaussians_;  //!< Gaussian components of this GMM.
	std::vector<double> priors_;                 //!< Priors for each Gaussian contained in this GMM.

private:
	Manifold manif_;
};
}  // namespace pglfd
