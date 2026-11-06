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
#include <Eigen/Core>
#include <cassert>
#include <vector>

namespace pglfd {
template<typename Manifold>
std::vector<typename Manifold::Vector> matrixToVector(const Eigen::MatrixXd &mat) {
	std::vector<typename Manifold::Vector> vec;
	if(mat.cols() == 1 && mat.rows() == Manifold::dim_manifold) {
		vec.push_back(mat.block<Manifold::dim_manifold, 1>(0, 0));
	} else if(mat.rows() == 1 && mat.cols() == Manifold::dim_manifold) {
		vec.push_back(mat.block<1, Manifold::dim_manifold>(0, 0));
	} else {
		assert((mat.cols() == Manifold::dim_manifold));
		for(int i = 0; i < mat.rows(); i++)
			vec.push_back(mat.block<1, Manifold::dim_manifold>(i, 0).transpose());
	}
	return vec;
}

template<typename Manifold>
Eigen::MatrixXd vectorToMatrix(const std::vector<typename Manifold::Vector> &vec) {
	Eigen::MatrixXd mat;
	mat.resize(vec.size(), Manifold::dim_manifold);
	for(int i = 0; i < vec.size(); i++) {
		mat.block<1, Manifold::dim_manifold>(i, 0) = vec[i].transpose();
	}
	return mat;
}
}  // namespace pglfd
