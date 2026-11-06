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

#ifdef NDEBUG
#undef NDEBUG
#endif
#include "pglfd/manif/jacobian.h"
#include <cassert>

using namespace pglfd;
int main() {
	// Cylindrical manifold
	typename CylindricalManifoldd::Vector data_cyl =
			manifold_from_r3_so3<CylindricalManifoldd>({1, 2, 3, 1, 0, 0, 0});
	typename CylindricalManifoldd::Vector data2_cyl =
			manifold_from_r3_so3<CylindricalManifoldd>({1, 2, 3, 0, 0, 0, 1});
	typename CylindricalManifoldd::TangentMatrix jac_cyl;
	jac_cyl << -0.4, 0.4472136, 0.0, 0.0, 0.0, -0.4, 0.2, 0.89442719, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0,
			1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 1.0;
	jac_cyl.transposeInPlace();
	typename CylindricalManifoldd::TangentMatrix jac2_cyl;
	jac2_cyl << -0.4, 0.4472136, 0.0, 0.0, 0.0, 0.4, 0.2, 0.89442719, 0.0, 0.0, 0.0, -0.2, 0.0, 0.0,
			1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 1.0;
	jac2_cyl.transposeInPlace();
	assert(
			(manifold_jacobian<CylindricalManifoldd>(data_cyl) - jac_cyl).isMuchSmallerThan(1.0, 1e-6));
	assert(
			(manifold_jacobian<CylindricalManifoldd>(data2_cyl) - jac2_cyl).isMuchSmallerThan(1.0, 1e-6));

	// Spherical manifold
	typename SphericalManifoldd::Vector data_sph =
			manifold_from_r3_so3<SphericalManifoldd>({1, 2, 3, 1, 0, 0, 0});
	typename SphericalManifoldd::Vector data2_sph =
			manifold_from_r3_so3<SphericalManifoldd>({1, 2, 3, 0, 0, 0, 1});
	typename SphericalManifoldd::TangentMatrix jac_sph;
	jac_sph << 2.56666136e-01, -2.11902111e-02, -7.14285714e-02, 0.00000000e00, 0.00000000e00,
			0.00000000e00, -2.11902111e-02, 2.24880820e-01, -1.42857143e-01, 0.00000000e00, 0.00000000e00,
			0.00000000e00, 2.67261242e-01, 5.34522484e-01, 8.01783726e-01, 0.00000000e00, 0.00000000e00,
			0.00000000e00, -1.93619966e-18, 2.14285714e-01, -1.42857143e-01, 1.00000000e00, 0.00000000e00,
			0.00000000e00, 2.14285714e-01, 1.61560638e-18, -7.14285714e-02, 0.00000000e00, 1.00000000e00,
			0.00000000e00, -1.42857143e-01, 7.14285714e-02, -2.59852131e-18, 0.00000000e00, 0.00000000e00,
			1.00000000e00;
	typename SphericalManifoldd::TangentMatrix jac2_sph;
	jac2_sph << 2.56666136e-01, -2.11902111e-02, -7.14285714e-02, 0.00000000e00, 0.00000000e00,
			0.00000000e00, -2.11902111e-02, 2.24880820e-01, -1.42857143e-01, 0.00000000e00, 0.00000000e00,
			0.00000000e00, 2.67261242e-01, 5.34522484e-01, 8.01783726e-01, 0.00000000e00, 0.00000000e00,
			0.00000000e00, -1.93619966e-18, 2.14285714e-01, -1.42857143e-01, 1.00000000e00, 0.00000000e00,
			0.00000000e00, -2.14285714e-01, -1.61560638e-18, 7.14285714e-02, 0.00000000e00, 1.00000000e00,
			0.00000000e00, 1.42857143e-01, -7.14285714e-02, 2.59852131e-18, 0.00000000e00, 0.00000000e00,
			1.00000000e00;
	assert((manifold_jacobian<SphericalManifoldd>(data_sph) - jac_sph).isMuchSmallerThan(1.0, 1e-6));
	assert(
			(manifold_jacobian<SphericalManifoldd>(data2_sph) - jac2_sph).isMuchSmallerThan(1.0, 1e-6));
	return 0;
}