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
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>
#include "pglfd/util/execution.h"

namespace pglfd {
/**
 * Calculate the weighted frechet mean of data.
 *
 * @param data The data to compute the mean of.
 * @param weights The weights to use during mean computation. Have to sum to 1.
 * @param eps Stopping criterion.
 * @param maxit Maximum number of iterations.
 * @return The mean value.
 */
template<typename Manifold>
inline typename Manifold::Vector frechet_mean(
		const std::vector<typename Manifold::Vector> &data,
		const std::vector<typename Manifold::Scalar> &weights = {},
		const typename Manifold::Scalar eps = 1e-10,
		const int maxit = 50) {
	if(data.size() == 0)
		throw std::runtime_error("Cannot calculate frechet mean of empty data!");

	std::vector<typename Manifold::Scalar> weights_copy = weights;
	if(weights_copy.size() == 0) {
		weights_copy.resize(data.size());
		std::fill(execution::unseq, weights_copy.begin(), weights_copy.end(), 1. / data.size());
	}
	assert(data.size() == weights_copy.size());
	Manifold manif;
	auto it_max = std::max_element(execution::unseq, weights_copy.begin(), weights_copy.end());
	int idx_max = std::distance(weights_copy.begin(), it_max);
	typename Manifold::Vector mu = data[idx_max];
	typename Manifold::Scalar diff = 1;
	int it = 0;
	std::vector<typename Manifold::TangentVector> weighted_logs(data.size());
	while(diff > eps) {
// Calculate logs
#pragma omp parallel for
		for(int i = 0; i < weighted_logs.size(); i++) {
			weighted_logs[i] = weights_copy[i] * manif.log(data[i], mu);
		}
		typename Manifold::TangentVector weighted_distance;
		weighted_distance.setZero();
		for(int i = 0; i < data.size(); i++) {
			weighted_distance += weighted_logs[i];
		}
		mu = manif.exp(weighted_distance, mu);
		diff = weighted_distance.transpose() * weighted_distance;

		it++;
		if(it > maxit) {
			std::cerr << "----------------------------------------------------" << std::endl;
			std::cerr << "Frechet mean calculation failed after " << it << " iterations." << std::endl;
			std::cerr << "Last diff: " << diff << ", weighted distance: " << weighted_distance.transpose()
								<< std::endl;
			std::cerr << "Weights:";
			for(const auto &weight: weights_copy) {
				std::cerr << " " << weight;
			}
			std::cerr << std::endl;
			std::cerr << "Data:" << std::endl;
			for(const auto &point: data) {
				std::cerr << point.transpose() << std::endl;
			}
			throw std::runtime_error("Frechet mean calculation did not converge");
		}
	}
	return mu;
}
}  // namespace pglfd
