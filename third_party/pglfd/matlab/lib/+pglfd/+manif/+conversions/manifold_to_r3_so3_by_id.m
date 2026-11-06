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

function [r3_so3] = manifold_to_r3_so3_by_id(manifold_id, manifold, point)
%MANIFOLD_TO_R3_SO3_BY_ID Convert a manifold element to ``R3*SO3``.
%   Transform an element from the manifold described by manifold to its
%   equivalent element on ``R3*SO3`` at the same origin.
%
%   :param manifold_id: The manifold id.
%   :param manifold: The manifold the vector resides in.
%   :param point: The vector on the manifold.
%   :return: The vector in ``R3*SO3``.

% check inputs
manifold.check_manifold_shape(point);

if manifold_id == 0
    r3_so3 = point;
elseif manifold_id == 1
    r3_so3 = zeros(7, size(point, 2));
    for i = 1:size(point, 2)
        % extract values
        cos_theta = point(1, i);
        sin_theta = point(2, i);
        r = point(3, i);
        z = point(4, i);

        % rotate so(3) part
        so3_addtangent = zeros(3, 1);
        so3_addtangent(3) = pglfd.manif.manifold.SO2Manifold.log(point(1:2, i), pglfd.manif.manifold.SO2Manifold.id_element_static);
        so3_add = pglfd.manif.manifold.SO3Manifold.exp(so3_addtangent, pglfd.manif.manifold.SO3Manifold.id_element_static);

        % construct element
        r3_so3(:, i) = [r * cos_theta; r * sin_theta; z; pglfd.geom.so3.quaternion_multiply(so3_add, point(5:end, i))];
    end
elseif manifold_id == 2
    r3_so3 = zeros(7, size(point, 2));
    for i = 1:size(point, 2)
        % extract values
        r = point(4, i);

        % rotate SO(3) part
        so3_add = pglfd.geom.so3.axis_angle_to_quaternion(pglfd.geom.s2.vector_to_axis_angle(point(1:3, i)));

        % construct element
        r3_so3(:, i) = [r * point(1:3, i); pglfd.geom.so3.quaternion_multiply(so3_add, point(5:end, i))];
    end
else
    error(['Unknown Manifold ' manifold.name]);
end
end
