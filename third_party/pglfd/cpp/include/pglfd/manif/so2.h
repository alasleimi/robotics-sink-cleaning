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
#include <cmath>  // IWYU pragma: keep
#include <map>
#include <numbers>
#include <string>
#include <tuple>
#include <vector>
#include "pglfd/geom/so2.h"
#include "pglfd/manif/manifold.h"

namespace pglfd {
/**
 * SO(2) manifold implementation.
 *
 * @param Scalar_ The data type to use for this manifold.
 */
template<typename Scalar_>
class SO2Manifold : public Manifold<Scalar_, 2, 1, true> {
public:
	using Scalar = Scalar_;                                  //!< Scalar type used on the manifold.
	using Vector = typename Manifold<Scalar, 2, 1>::Vector;  //!< On-manifold vector type.
	using TangentVector = typename Manifold<Scalar, 1, 1>::TangentVector;   //!< Tangent vector type.
	using TangentMatrix = typename Manifold<Scalar, 1, 1>::TangentMatrix;   //!< Tangent matrix type.
	using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;  //!< Dynamic matrix type.

	std::string name() const override {
		return "SO2";
	}

	Vector id_elem() const override {
		Vector id;
		id << 0, 1;
		return id;
	}

	Vector normalize(const Vector &vector) const override {
		return SO2<Scalar>(vector).normalize().toVec();
	}

	Vector exp(const TangentVector &tangent, const Vector &base) const override {
		SO2<Scalar> so2_base = SO2<Scalar>(base);
		return SO2<Scalar>(so2_base * SO2<Scalar>::exp(tangent)).toVec();
	}

	TangentVector log(const Vector &point, const Vector &base) const override {
		SO2<Scalar> so2_point = SO2<Scalar>(point);
		SO2<Scalar> so2_base = SO2<Scalar>(base);
		return (so2_base.inverse() * so2_point).log();
	}

	std::tuple<MatrixX, std::vector<int>> continuousLog(
			const MatrixX &tangents,
			const std::map<int, int> &vias = {},
			bool start_low = true) const override {
		Scalar add_glob = 0;
		MatrixX res = tangents;
		for(int i = 0; i < res.cols(); i++) {
			res(0, i) += add_glob;
			if(std::abs(res(0, i) + 2 * std::numbers::pi - res(0, i - 1))
				 < std::abs(res(0, i) - res(0, i - 1))) {
				res(0, i) += 2 * std::numbers::pi;
				add_glob += 2 * std::numbers::pi;
			} else if(
					std::abs(res(0, i) - 2 * std::numbers::pi - res(0, i - 1))
					< std::abs(res(0, i) - res(0, i - 1))) {
				res(0, i) -= 2 * std::numbers::pi;
				add_glob -= 2 * std::numbers::pi;
			}
		}
		return {res, {}};
	}

	std::vector<Vector> action(
			const std::vector<Vector> &elems, const Vector &g, const Vector &h) const override {
		SO2<Scalar> so2_g = SO2<Scalar>(g);
		SO2<Scalar> so2_h = SO2<Scalar>(h);
		std::vector<Vector> res;
		for(const auto &elem: elems) {
			SO2<Scalar> so2_elem = SO2<Scalar>(elem);
			res.push_back(SO2<Scalar>(so2_h * so2_g.inverse() * so2_elem).toVec());
		}
		return res;
	}

	TangentMatrix parallelTransportTrafo(const Vector &g, const Vector &h) const override {
		return TangentMatrix::Identity();
	}
};

using SO2Manifoldd = SO2Manifold<double>;
}  // namespace pglfd