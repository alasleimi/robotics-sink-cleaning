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

function [parallel_transport_trafo_val] = parallel_transport(g, h)
%PARALLEL_TRANSPORT The parallel transport matrix from g to h.
%
%   :param g: Tangent space of origin.
%   :param h: Tangent space of target.
%   :return: Parallel transport matrix from g to h.

Reg = pglfd.geom.so3.axis_angle_to_rotmat(pglfd.geom.s2.vector_to_axis_angle(g));
Reh = pglfd.geom.so3.axis_angle_to_rotmat(pglfd.geom.s2.vector_to_axis_angle(h));

v = Reg * [pglfd.manif.manifold.S2Manifold.log(h, g); 0];
m = norm(v);
if m < 1e-6
    Rgh = eye(3);
else
    u = v / m;
    Rgh = eye(3) ...
        -sin(m) * (g * u') ...
        +(cos(m) - 1) * (u * u');
end

B = eye(3, 2);
parallel_transport_trafo_val = B' * Reh' * Rgh * Reg * B;

% ensure proper rotation
if sign(trace(parallel_transport_trafo_val)) == -1
    parallel_transport_trafo_val = -parallel_transport_trafo_val;
end
end
