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
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include "pglfd/geom/so3.h"
#include "pglfd/manif/conversions.h"
#include "pglfd/manif/manifold.h"

namespace pglfd {
/**
 * Jacobian mapping from Cartesian space into tangent space.
 *
 * Calculate the Jacobian to transform a velocity in the tangent space of ``R3*SO3`` to a velocity
 * in tangent space of ``Manifold`` at ``point``.
 *
 * @param Manifold The manifold.
 * @param point The point at which to calculate the Jacobian.
 * @param eps Epsilon value for small radii.
 * @return The Jacobian.
 */
template<typename Manifold>
inline typename Manifold::TangentMatrix manifold_jacobian(
		const typename Manifold::Vector &point, typename Manifold::Scalar eps = 1e-10);

// R3*SO(3) - the identity Matrix
template<>
inline SE3Manifold<double>::TangentMatrix manifold_jacobian<SE3Manifold<double>>(
		const typename SE3Manifold<double>::Vector &point, double eps) {
	return SE3Manifold<double>::TangentMatrix::Identity();
}

// SO(2)*R2*SO(3)
template<>
inline CylindricalManifold<double>::TangentMatrix manifold_jacobian<CylindricalManifold<double>>(
		const typename CylindricalManifold<double>::Vector &point, double eps) {
	typename CylindricalManifold<double>::TangentMatrix jacobian =
			CylindricalManifold<double>::TangentMatrix::Identity();
	typename SE3Manifold<double>::Vector point_r3_so3 =
			manifold_to_r3_so3<CylindricalManifold<double>>(point);

	// Calculate radius
	double r = point_r3_so3.head(2).norm();

	if(r > eps) {
		// for theta = atan2(y, x)
		jacobian(0, 0) = -point_r3_so3[1] / (r * r);
		jacobian(0, 1) = point_r3_so3[0] / (r * r);

		// for r = sqrt(x ^ 2 + y ^ 2)
		jacobian(1, 0) = point_r3_so3[0] / r;
		jacobian(1, 1) = point_r3_so3[1] / r;
	} else {
		jacobian.template block<2, 2>(0, 0).setZero();
	}

	// for theta = atan2(y, x) -> this has an influence on the rotational
	// velocity around z!
	jacobian(5, 0) = -jacobian(0, 0);
	jacobian(5, 1) = -jacobian(0, 1);

	// rotate with the current orientation
	Eigen::Matrix<double, 3, 3> rotm =
			SO3<double>(point.template tail<4>()).toQuat().toRotationMatrix().transpose();
	jacobian.template block<3, 3>(3, 0) = rotm * jacobian.template block<3, 3>(3, 0);

	return jacobian;
}

// S2*R1*SO(3)
template<>
inline SphericalManifold<double>::TangentMatrix manifold_jacobian<SphericalManifold<double>>(
		const typename SphericalManifold<double>::Vector &point, double eps) {
	typename SphericalManifold<double>::TangentMatrix jacobian =
			SphericalManifold<double>::TangentMatrix::Identity();
	typename SE3Manifold<double>::Vector point_r3_so3 =
			manifold_to_r3_so3<SphericalManifold<double>>(point);

	// Calculate radius
	double r = point_r3_so3.head(3).norm();

	// positional part, jacobian at origin -> angular part depends on radius
	jacobian(0, 0) /= r;
	jacobian(1, 1) /= r;
	// rotate x, y, z coordinates to the north pole (transpose of original rotmat)
	Eigen::Matrix<double, 3, 3> rotm =
			Eigen::Quaternion<double>((S2<double>(point.template head<3>())).axis_angle())
					.toRotationMatrix()
					.transpose();
	jacobian.template block<3, 3>(0, 0) = jacobian.template block<3, 3>(0, 0) * rotm;  // compose

	// first two angles also have an influence on the orientation
	jacobian(3, 0) = jacobian(1, 0);
	jacobian(3, 1) = jacobian(1, 1);
	jacobian(3, 2) = jacobian(1, 2);
	jacobian(4, 0) = -jacobian(0, 0);
	jacobian(4, 1) = -jacobian(0, 1);
	jacobian(4, 2) = -jacobian(0, 2);

	// rotate with the current orientation
	rotm = SO3<double>(point.template tail<4>()).toQuat().toRotationMatrix().transpose();
	jacobian.template block<3, 3>(3, 0) = rotm * jacobian.template block<3, 3>(3, 0);

	return jacobian;
}
}  // namespace pglfd