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

% check conversions
assert(all(all( ...
    pglfd.geom.so3.quaternion_to_rotmat([0.1714481 0.2571721 0.085724 0.9471639]')- ...
    [0.8530278, -0.0742061, 0.5165628; ...
    0.2505728, 0.9265139, -0.2806872; ...
    -0.4577738, 0.3688705, 0.8089361] < 1e-6)));

assert(all( ...
    pglfd.geom.so3.rotmat_to_quaternion( ...
    [0.8530278, -0.0742061, 0.5165628; ...
    0.2505728, 0.9265139, -0.2806872; ...
    -0.4577738, 0.3688705, 0.8089361])- ...
    [0.1714481 0.2571721 0.085724 0.9471639]' < 1e-6));

for anglex = 1:5:50
    for angley = 1:5:50
        for anglez = 1:5:50
            angle = (anglex + angley + anglez) / 180 * pi;
            axang = [anglex angley anglez angle]';
            axang(1:3) = axang(1:3) / norm(axang(1:3));

            quat = pglfd.geom.so3.axis_angle_to_quaternion(axang);
            rotm = pglfd.geom.so3.axis_angle_to_rotmat(axang);

            assert(all(all(pglfd.geom.so3.quaternion_to_rotmat(quat)-rotm < 1e-8)));
            assert(all(pglfd.geom.so3.rotmat_to_quaternion(rotm)-quat < 1e-8));
        end
    end
end

assert(all(pglfd.geom.so3.rotmat_to_quaternion([0 0 1; 0 -1 0; 1 0 0])-[-0.7071 0 -0.7071 0]' < 1e-4));
