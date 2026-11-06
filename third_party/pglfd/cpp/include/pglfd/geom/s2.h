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
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <numbers>

namespace pglfd {
/**
 * S2 Manifold implementation.
 *
 * @param Scalar The data type to use.
 */
template<typename Scalar>
class S2 {
public:
	using Vector = Eigen::Matrix<Scalar, 3, 1>;         //!< On-manifold vector type.
	using Tangent = Eigen::Matrix<Scalar, 2, 1>;        //!< Tangent vector type.
	using TangentMatrix = Eigen::Matrix<Scalar, 2, 2>;  //!< Tangent matrix type.

	/**
	 * S2 element constructor.
	 *
	 * @param vec The S2 element.
	 */
	explicit S2(const Vector& vec) : vec_(vec) {
		assert(std::abs(vec_.norm() - 1) < 0.1);
	}

	/**
	 * Normalize the S2 element.
	 */
	S2<Scalar>& normalize() {
		assert(std::abs(vec_.norm() - 1) < 0.1);
		vec_ /= vec_.norm();
		return *this;
	}

	/**
	 * Get the S2 vector.
	 *
	 * @return The S2 vector.
	 */
	constexpr const Vector& toVec() const {
		return vec_;
	}

	/**
	 * Get the inverse element.
	 *
	 * @return The inverse S2 element.
	 */
	constexpr S2<Scalar> inverse() const {
		// Calculate the inverse: same z coordiante, invert x and y
		Vector new_vec = vec_;
		new_vec[0] *= -1;
		new_vec[1] *= -1;
		return S2<Scalar>(new_vec);
	}

	/**
	 * Get the angle-axis representation to rotate [0, 0, 1] into us.
	 *
	 * @param eps Epsilon value to check for a rotation.
	 * @return The angle axis representation.
	 */
	constexpr Eigen::AngleAxis<Scalar> axis_angle(double eps = 1e-10) const {
		Eigen::AngleAxis<Scalar> axang = Eigen::AngleAxis<Scalar>(0, Eigen::Vector<Scalar, 3>::UnitZ());
		double len = vec_.template head<2>().norm();
		if(len < eps) {
			if(vec_[2] < 0) {
				axang = Eigen::AngleAxis<Scalar>(std::numbers::pi, Eigen::Vector<Scalar, 3>::UnitX());
			}
		} else {
			Eigen::Vector<Scalar, 3> axis;
			axis[0] = -vec_[1];
			axis[1] = vec_[0];
			axis[2] = 0;
			axang.axis() = axis / axis.norm();

			// angle
			axang.angle() = std::acos(vec_[2]);
		}
		return axang;
	}

	/**
	 * Combination operation. TODO this is not really a group operation
	 *
	 * @param rhs The other element to multiply with.
	 * @return The multiplication operation of this and rhs.
	 */
	constexpr S2<Scalar> operator*(const S2<Scalar>& rhs) const {
		Eigen::Matrix3<Scalar> rotmat = axis_angle().toRotationMatrix();
		return S2<Scalar>(rotmat * rhs.toVec());
	}

	/**
	 * Exponential operation.
	 *
	 * @param tangent The tangent vector to exponentiate.
	 * @param eps Lengths smaller than this threshold get projected to the origin.
	 * @return The manifold element.
	 */
	static constexpr S2<Scalar> exp(const Tangent& tangent, double eps = 1e-10) {
		Vector new_vec;
		new_vec << 0, 0, 1;
		double len = tangent.norm();
		if(len > eps) {
			new_vec.template head<2>() = std::sin(len) / len * tangent;
			new_vec[2] = std::cos(len);
		}
		return S2<Scalar>(new_vec);
	}

	/**
	 * Logarithm operation.
	 *
	 * @return The tangent vector at the origin.
	 */
	constexpr Tangent log(double eps = 1e-10) const {
		Tangent tangent;
		tangent.setZero();
		if(std::abs(1 - vec_[2]) > eps) {
			tangent = std::acos(vec_[2]) / vec_.template head<2>().norm() * vec_.template head<2>();
		}
		return tangent;
	}

	/**
	 * Calculate the parallel transport operation
	 *
	 * @param h The element to transport to.
	 * @param eps Epsilon value to check for rotation.
	 * @return The parallel transport from us to h.
	 */
	constexpr TangentMatrix parallelTransport(S2<Scalar> h, double eps = 1e-10) const {
		Eigen::Matrix3<Scalar> Reg = axis_angle(eps).toRotationMatrix();
		Eigen::Matrix3<Scalar> Reh = h.axis_angle(eps).toRotationMatrix();
		Eigen::Matrix3<Scalar> Rgh;
		Eigen::Vector3<Scalar> vec;
		vec << (inverse() * h).log(), 0;
		Eigen::Vector3<Scalar> v = Reg * vec;
		double m = v.norm();
		if(m < eps) {
			Rgh.setIdentity();
		} else {
			Eigen::Vector3<Scalar> u = v / m;
			Rgh = Eigen::Matrix3<Scalar>::Identity() - std::sin(m) * vec_ * u.transpose()
						+ (std::cos(m) - 1) * u * u.transpose();
		}
		Eigen::Matrix<Scalar, 3, 2> B;
		B.setZero();
		B(0, 0) = 1;
		B(1, 1) = 1;
		TangentMatrix R = B.transpose() * Reh.transpose() * Rgh * Reg * B;
		if(R.trace() < 0) {
			R = -R;
		}
		return R;
	}

private:
	Vector vec_;
};
using S2d = S2<double>;
}  // namespace pglfd
