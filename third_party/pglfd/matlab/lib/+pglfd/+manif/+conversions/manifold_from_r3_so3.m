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

function [point] = manifold_from_r3_so3(manifold, r3_so3)
%MANIFOLD_FROM_R3_SO3 Convert a ``R3*SO3`` element to manifold.
%   Transform an element from ``R3*SO3`` to its equivalent elment on the
%   manifold described by description at the same origin.
%
%   :param manifold: The manifold the vector should be converted to.
%   :param r3_so3: The vector on ``R3*SO3``.
%   :return: The vector in manifold coordinates.

assert(size(r3_so3, 1) == 7, 'Point shape mismatch');

if strcmp(manifold.name, 'R3*SO3')
    point = r3_so3;
elseif strcmp(manifold.name, 'S2*R1*SO3')
    point = zeros(manifold.dim_manifold, size(r3_so3, 2));
    for i = 1:size(r3_so3, 2)
        % extract position part
        r = norm(r3_so3(1:3, i));

        % construct element
        point(:, i) = [r3_so3(1:3, i) / r; r; r3_so3(4:end, i)];

        % fix for degenerate case with r < eps
        if r < 1e-10
            point(1:3, i) = pglfd.manif.manifold.S2Manifold.id_element_static;
        end

        % rotate so(3) part
        so3_add = pglfd.geom.so3.quaternion_inverse(pglfd.geom.so3.axis_angle_to_quaternion(pglfd.geom.s2.vector_to_axis_angle(point(1:3, i))));
        point(5:end, i) = pglfd.geom.so3.quaternion_multiply(so3_add, point(5:end, i));
    end
elseif strcmp(manifold.name, 'SO2*R2*SO3')
    point = zeros(manifold.dim_manifold, size(r3_so3, 2));
    for i = 1:size(r3_so3, 2)
        % extract position part
        r = norm(r3_so3(1:2, i));

        % construct element
        point(:, i) = [r3_so3(1, i) / r; r3_so3(2, i) / r; r; r3_so3(3:end, i)];

        % fix for degenerate case with r < eps
        if r < 1e-10
            point(1:2, i) = pglfd.manif.manifold.SO2Manifold.id_element_static;
            if r3_so3(2, i) > r3_so3(1, i)
                point(1, i) = 0;
                point(2, i) = 1;
            end
        end

        % rotate so(3) part
        so3_addtangent = zeros(3, 1);
        so3_addtangent(3) = -pglfd.manif.manifold.SO2Manifold.log(point(1:2, i), pglfd.manif.manifold.SO2Manifold.id_element_static);
        so3_add = pglfd.manif.manifold.SO3Manifold.exp(so3_addtangent, pglfd.manif.manifold.SO3Manifold.id_element_static);
        point(5:end, i) = pglfd.geom.so3.quaternion_multiply(so3_add, point(5:end, i));
    end
else
    error(['Unknown Manifold ' manifold.name]);
end
end
