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
#include <cmath>
#include "pglfd/geom/so3.h"

namespace pglfd {
/**
 * SE(3) Manifold implementation.
 *
 * @param Scalar The data type to use.
 */
template<typename Scalar>
class SE3 {
public:
	using Vector = Eigen::Matrix<Scalar, 7, 1>;  //!< On-manifold vector type.

	/**
	 * SE(3) element constructor.
	 *
	 * @param vec The 7-d pose.
	 */
	explicit SE3(const Vector &vec) : vec_(vec) {}

	/**
	 * Get the 7-d pose.
	 *
	 * @return The 7-d pose as vector.
	 */
	constexpr const Vector &toVec() const {
		return vec_;
	}

	/**
	 * Get the inverse element.
	 *
	 * @return The inverse SE(3) element.
	 */
	constexpr SE3<Scalar> inverse() const {
		Vector res = vec_;
		SO3<Scalar> so3_inv = SO3<Scalar>(toVec().tail(4)).inverse();
		res.tail(4) = so3_inv.toVec();
		res.head(3) = -(so3_inv.toQuat() * vec_.head(3));
		return SE3<Scalar>(res);
	}

	/**
	 * Group operation.
	 *
	 * @param rhs The other element to multiply with.
	 * @return The group operation of this and rhs.
	 */
	constexpr SE3<Scalar> operator*(const SE3<Scalar> &rhs) const {
		Vector res;
		SO3<Scalar> so3_1(toVec().tail(4));
		SO3<Scalar> so3_2(rhs.toVec().tail(4));
		res.head(3) = vec_.head(3) + so3_1.toQuat() * rhs.toVec().head(3);
		res.tail(4) = (so3_1 * so3_2).toVec();
		return SE3<Scalar>(res);
	}

private:
	Vector vec_;
};

using SE3d = SE3<double>;
}  // namespace pglfd
