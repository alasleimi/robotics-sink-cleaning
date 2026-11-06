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

% Test the cylindrical manifold
pt_cyl = [1 2 3 1 0 0 0]';
man_cyl = pglfd.manif.util.get_productmanifold('SO2*R2*SO3');
assert(all(all(pglfd.manif.conversions.manifold_to_r3_so3(man_cyl, ...
    pglfd.manif.conversions.manifold_from_r3_so3(man_cyl, pt_cyl))-pt_cyl < 1e-12)));

% Test the spherical manifold
pt_spher = [1 2 3 1 0 0 0]';
man_spher = pglfd.manif.util.get_productmanifold('S2*R1*SO3');
assert(all(all(pglfd.manif.conversions.manifold_to_r3_so3(man_spher, ...
    pglfd.manif.conversions.manifold_from_r3_so3(man_spher, pt_spher))-pt_spher < 1e-12)));

% Test a batch operation
pt_cyl_batch = [pt_cyl pt_cyl];
assert(all(all(pglfd.manif.conversions.manifold_to_r3_so3(man_cyl, ...
    pglfd.manif.conversions.manifold_from_r3_so3(man_cyl, pt_cyl_batch))-pt_cyl_batch < 1e-12)));

pt_spher_batch = [pt_spher pt_spher];
assert(all(all(pglfd.manif.conversions.manifold_to_r3_so3(man_spher, ...
    pglfd.manif.conversions.manifold_from_r3_so3(man_spher, pt_spher_batch))-pt_spher_batch < 1e-12)));
