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
#include <algorithm>  // IWYU pragma: keep
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
#include "pglfd/manif/util.h"
#include "pglfd/util/execution.h"

namespace pglfd {
/**
 * Riemannian KMeans implementation.
 *
 * @param Manifold The manifold of the data.
 */
template<typename Manifold>
class KMeans {
public:
	/**
	 * Constructor of the kmeans.
	 *
	 * @param clusters The clustered data.
	 * @param cluster_centers The precomputed cluster centers - if empty, the mean will be recomputed.
	 **/
	explicit KMeans(
			const std::vector<std::vector<typename Manifold::Vector>> &clusters,
			const std::vector<typename Manifold::Vector> &cluster_centers = {}) :
			cluster_centers_(cluster_centers), clusters_(clusters) {
		if(cluster_centers_.size() != clusters_.size()) {
			cluster_centers_.resize(clusters_.size());
			std::transform(
					execution::unseq,
					clusters_.begin(),
					clusters_.end(),
					cluster_centers_.begin(),
					[](const auto &cluster) { return frechet_mean<Manifold>(cluster); });
		}
	}

	/**
	 * Initialize the kmeans algorithm.
	 *
	 * @param k Number of clusters to use.
	 * @param data The data to fit.
	 * @param maxit Maximum number of iterations to perform.
	 * @return Fitted kmeans.
	 **/
	static KMeans fit(int k, const std::vector<typename Manifold::Vector> &data, int maxit = 100) {
		Manifold manif;

		// Initialize means
		std::vector<typename Manifold::Vector> cluster_centers;
		for(int i = 0; i < k; i++) {
			cluster_centers.push_back(data[std::rand() % data.size()]);
		}
		std::vector<std::vector<typename Manifold::Vector>> clusters(cluster_centers.size());

		std::vector<int> id_old(data.size());
		std::vector<int> id_min(data.size());
		int it = 0;
		while(true) {
			// Calculate distances to cluster centers
#pragma omp parallel for
			for(int i = 0; i < data.size(); i++) {
				typename Manifold::Scalar min_dist = std::numeric_limits<double>::max();
				for(int j = 0; j < k; j++) {
					typename Manifold::Scalar dist = manif.log(data[i], cluster_centers[j]).squaredNorm();
					if(dist < min_dist) {
						id_min[i] = j;
						min_dist = dist;
					}
				}
			}

			// Update clusters
			for(int i = 0; i < k; i++) {
				clusters[i].clear();  // Only clear but do not reduce the size
				for(int j = 0; j < data.size(); j++) {
					if(id_min[j] == i) {
						clusters[i].push_back(data[j]);
					}
				}
				if(clusters[i].size() > 0) {
					cluster_centers[i] = frechet_mean<Manifold>(clusters[i]);
				} else {
					cluster_centers[i] = data[std::rand() % data.size()];
					std::cerr << "[WARNING] k-means replaced empty cluster by random value" << std::endl;
				}
			}

			// Check for convergence
			bool all_same = true;
			for(int i = 0; i < data.size(); i++) {
				if(id_old[i] != id_min[i])
					all_same = false;
			}
			if(all_same) {
				break;
			} else {
				id_old = id_min;
			}

			it++;
			if(it > maxit) {
				throw std::runtime_error("KMeans fitting did not converge");
			}
		}

		return KMeans(clusters, cluster_centers);
	}

	std::vector<typename Manifold::Vector> cluster_centers_;  //!< Centers of the computed clusters.
	std::vector<std::vector<typename Manifold::Vector>> clusters_;  //!< Data of the clusters.
};
}  // namespace pglfd
