% Copyright (c) 2026 Maximilian Mühlbauer
%
% This file is part of pglfd.
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

% S1
man = pglfd.manif.util.get_productmanifold('SO2');
assert(man.dim_manifold == 2);
assert(man.dim_tangent == 1);

% S2
man = pglfd.manif.util.get_productmanifold('S2');
assert(man.dim_manifold == 3);
assert(man.dim_tangent == 2);

% S3
man = pglfd.manif.util.get_productmanifold('SO3');
assert(man.dim_manifold == 4);
assert(man.dim_tangent == 3);

% R3*S3
man = pglfd.manif.util.get_productmanifold('R3*SO3');
assert(man.dim_manifold == 7);
assert(man.dim_tangent == 6);

% S1*R2*S3
man = pglfd.manif.util.get_productmanifold('SO2*R2*SO3');
assert(man.dim_manifold == 8);
assert(man.dim_tangent == 6);

% S2*R1*S3
man = pglfd.manif.util.get_productmanifold('S2*R1*SO3');
assert(man.dim_manifold == 8);
assert(man.dim_tangent == 6);

% S1*S1*R1*S3
man = pglfd.manif.util.get_productmanifold('SO2*SO2*R1*SO3');
assert(man.dim_manifold == 9);
assert(man.dim_tangent == 6);
