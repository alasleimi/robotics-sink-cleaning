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
#include "pglfd/manif/product.h"
#include <cassert>

using namespace pglfd;
int main() {
	SE3Manifoldd manif;
	SE3Manifoldd::Vector vec;
	SE3Manifoldd::TangentVector tan_vec;
	SE3Manifoldd::TangentMatrix mat;
	assert(SE3Manifoldd::dim_manifold == 7);
	assert(SE3Manifoldd::dim_tangent == 6);
	vec << 0, 0, 0, 0, 0, 0, 1;
	assert((manif.id_elem() - vec).isMuchSmallerThan(1e-5));
	assert((manif.action({vec}, vec, vec)[0] - vec).isMuchSmallerThan(1e-5));
	mat.setIdentity();
	assert((manif.parallelTransportTrafo(vec, vec) - mat).isMuchSmallerThan(1e-5));
	assert((manif.parallelTransportVector(tan_vec, vec, vec) - tan_vec).isMuchSmallerThan(1e-5));
	assert((manif.parallelTransportMatrix(mat, vec, vec) - mat).isMuchSmallerThan(1e-5));
	assert((std::tuple_element_t<0, decltype(manif.template submanifold<0>())>::dim_manifold == 3));
	assert((std::tuple_element_t<0, decltype(manif.template submanifold<1>())>::dim_manifold == 4));

	return 0;
}