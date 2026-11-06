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

function [jacobian, singular] = manifold_jacobian_by_id(manifold_id, manifold, point, eps)
%MANIFOLD_JACOBIAN_BY_ID Jacobian from R3*SO3 velocity to manifold at point.
%   Calculate the Jacobian to transform a velocity in the tangent space
%   of R3*SO3 to a velocity in tangent space of manifold at point.
%
%   :param manifold_id: The manifold id.
%   :param manifold: Target manifold for the Jacobian.
%   :param point: The point at which to calculate the Jacobian.
%   :param eps: Epsilon value for small radii.
%   :return: The jacobian and whether it is singular.

% reset singular
singular = false;

% check input shape
manifold.check_manifold_shape(point);

if manifold_id == 0
    jacobian = eye(6);
elseif manifold_id == 1
    jacobian = eye(6);
    point_r3_so3 = pglfd.manif.conversions.manifold_to_r3_so3_by_id(manifold_id, manifold, point);
    r_sq = point_r3_so3(1)^2 + point_r3_so3(2)^2;
    r = sqrt(r_sq);
    singular = r <= eps;

    % theta = atan2(y,x)
    if ~singular
        jacobian(1, 1) = -point_r3_so3(2) / r_sq;
        jacobian(1, 2) = point_r3_so3(1) / r_sq;
    else
        jacobian(1, 1) = 0;
    end

    % r = sqrt(x^2 + y^2)
    if ~singular
        jacobian(2, 1) = point_r3_so3(1) / r;
        jacobian(2, 2) = point_r3_so3(2) / r;
    else
        jacobian(2, 2) = 0;
    end

    % theta = atan2(y,x) -> this has an influence on the rotational
    % velocity around z!
    jacobian(6, 1) = -jacobian(1, 1);
    jacobian(6, 2) = -jacobian(1, 2);
    % rotate with the current orientation
    rotm = pglfd.geom.so3.quaternion_to_rotmat(point(5:end))';
    jacobian(4:6, 1:3) = rotm * jacobian(4:6, 1:3);
elseif manifold_id == 2
    jacobian = eye(6);
    point_r3_so3 = pglfd.manif.conversions.manifold_to_r3_so3_by_id(manifold_id, manifold, point);

    % calculate radius
    r = sqrt(point_r3_so3(1)^2+point_r3_so3(2)^2+point_r3_so3(3)^2);
    singular = r <= eps;

    % first two angles, jacobian at origin
    if ~singular
        jacobian_origin = eye(3);
        jacobian_origin(1:2, 1:2) = jacobian_origin(1:2, 1:2) * 1 / r;
        rotmat = pglfd.geom.so3.axis_angle_to_rotmat(pglfd.geom.s2.vector_to_axis_angle(point(1:3)));
        jacobian(1:3, 1:3) = (rotmat * jacobian_origin)';
    else
        jacobian(1:3, 1:3) = 0;
    end

    % first two angles also have an influence on the orientation
    jacobian(4, 1) = jacobian(2, 1);
    jacobian(4, 2) = jacobian(2, 2);
    jacobian(4, 3) = jacobian(2, 3);
    jacobian(5, 1) = -jacobian(1, 1);
    jacobian(5, 2) = -jacobian(1, 2);
    jacobian(5, 3) = -jacobian(1, 3);
    % rotate with the current orientation
    rotm = pglfd.geom.so3.quaternion_to_rotmat(point(5:end))';
    jacobian(4:6, 1:3) = rotm * jacobian(4:6, 1:3);
else
    error(['Unknown Manifold id ' manifold_id]);
end
end
