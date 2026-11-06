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
#include <map>
#include <string>
#include <tuple>
#include <vector>
#include "pglfd/geom/s2.h"
#include "pglfd/manif/manifold.h"

namespace pglfd {
/**
 * S2 manifold implementation.
 *
 * @param Scalar_ The data type to use for this manifold.
 */
template<typename Scalar_>
class S2Manifold : public Manifold<Scalar_, 3, 2> {
public:
	using Scalar = Scalar_;                                  //!< Scalar type used on the manifold.
	using Vector = typename Manifold<Scalar, 3, 2>::Vector;  //!< On-manifold vector type.
	using TangentVector = typename Manifold<Scalar, 3, 2>::TangentVector;   //!< Tangent vector type.
	using TangentMatrix = typename Manifold<Scalar, 3, 2>::TangentMatrix;   //!< Tangent matrix type.
	using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;  //!< Dynamic matrix type.

	std::string name() const override {
		return "S2";
	}

	Vector id_elem() const override {
		Vector id;
		id << 0, 0, 1;
		return id;
	}

	Vector normalize(const Vector &vector) const override {
		return S2<Scalar>(vector).normalize().toVec();
	}

	Vector exp(const TangentVector &tangent, const Vector &base) const override {
		S2<Scalar> s2_base = S2<Scalar>(base);
		return S2<Scalar>(s2_base * S2<Scalar>::exp(tangent)).toVec();
	}

	TangentVector log(const Vector &point, const Vector &base) const override {
		S2<Scalar> s2_point = S2<Scalar>(point);
		S2<Scalar> s2_base = S2<Scalar>(base);
		return (s2_base.inverse() * s2_point).log();
	}

	std::tuple<MatrixX, std::vector<int>> continuousLog(
			const MatrixX &tangents,
			const std::map<int, int> &vias = {},
			bool start_low = true) const override {
		// TODO: implement discontinuity finding on S2
		return {tangents, {}};
	}

	std::vector<Vector> action(
			const std::vector<Vector> &elems, const Vector &g, const Vector &h) const override {
		S2<Scalar> s2_g = S2<Scalar>(g);
		S2<Scalar> s2_h = S2<Scalar>(h);
		std::vector<Vector> res;
		for(const auto &elem: elems) {
			S2<Scalar> s2_elem = S2<Scalar>(elem);
			res.push_back(S2<Scalar>(s2_h * s2_g.inverse() * s2_elem).toVec());
		}
		return res;
	}

	TangentMatrix parallelTransportTrafo(const Vector &g, const Vector &h) const override {
		return S2<Scalar>(g).parallelTransport(S2<Scalar>(h));
	}
};

using S2Manifoldd = S2Manifold<double>;
}  // namespace pglfd