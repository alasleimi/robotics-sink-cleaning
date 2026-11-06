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

% generate a pose
pose = [1 2 3 0 0 0 1];

% Test the cylindrical manifold
man = pglfd.manif.util.get_productmanifold('SO2*R2*SO3');
pose_cyl = pglfd.manif.conversions.manifold_from_r3_so3(man, pose');
pose_cyl(5:8) = [0 0 0 1];
r = pose_cyl(3);
jac_cyl = eye(6);
jac_cyl(1, 1) = -1 / r;
angle = atan2(pose_cyl(2), pose_cyl(1)) - pi / 2;
rotmat = [[cos(angle) -sin(angle)]; [sin(angle) cos(angle)]];
jac_cyl(1:2, 1:2) = jac_cyl(1:2, 1:2) * rotmat';
jac_cyl(6, 1) = 1 / r;
jac_cyl(5:6, 1:2) = jac_cyl(5:6, 1:2) * rotmat';
assert(all(all(pglfd.manif.jacobian.manifold_jacobian(man, pose_cyl, 1e-5)-jac_cyl < 1e-8)));

% Test the spherical manifold
man = pglfd.manif.util.get_productmanifold('S2*R1*SO3');
pose_sph = pglfd.manif.conversions.manifold_from_r3_so3(man, pose');
pose_sph(5:8) = [0 0 0 1];
r = pose_sph(4);
r_align = pglfd.geom.so3.axis_angle_to_rotmat(pglfd.geom.s2.vector_to_axis_angle(pose_sph(1:3)));
jac_sph = eye(6);
jac_sph(1:3, 1:3) = diag([1 / r 1 / r 1]) * r_align';
jac_sph(4:6, 1:3) = [[0 1 / r 0]; [-1 / r 0 0]; [0 0 0]] * r_align';
assert(all(all(pglfd.manif.jacobian.manifold_jacobian(man, pose_sph, 1e-5)-jac_sph < 1e-8)));
