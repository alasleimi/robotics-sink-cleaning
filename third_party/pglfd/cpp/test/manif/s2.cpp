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
#include "pglfd/manif/s2.h"
#include <cassert>

using namespace pglfd;
int main() {
	S2Manifoldd manif;
	S2Manifoldd::Vector vec;
	S2Manifoldd::TangentVector tan_vec;
	S2Manifoldd::TangentMatrix mat;
	assert(S2Manifoldd::dim_manifold == 3);
	assert(S2Manifoldd::dim_tangent == 2);
	vec << 0, 0, 1;
	tan_vec << 1, 2;
	assert((manif.id_elem() - vec).isMuchSmallerThan(1e-5));
	assert((manif.action({vec}, vec, vec)[0] - vec).isMuchSmallerThan(1e-5));
	mat.setIdentity();
	assert((manif.parallelTransportTrafo(vec, vec) - mat).isMuchSmallerThan(1e-5));
	assert((manif.parallelTransportVector(tan_vec, vec, vec) - tan_vec).isMuchSmallerThan(1e-5));
	assert((manif.parallelTransportMatrix(mat, vec, vec) - mat).isMuchSmallerThan(1e-5));

	return 0;
}
