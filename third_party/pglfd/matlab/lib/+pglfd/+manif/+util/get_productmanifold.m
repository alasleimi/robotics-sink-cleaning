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

function [manifold] = get_productmanifold(description, dim_manifold, dim_tangent)
%GET_PRODUCTMANIFOLD Get a product manifold described by description.
%   This method creates a product manifold from a description, e.g.
%   ``R3*SO3``. A few product manifolds are predefined for code generation,
%   inside Matlab, arbitrary manifolds can be generated. For code
%   generation, dim_manifold and dim_tangent have to be specified as well,
%   they can be obtained beforehand using the :mat:func:`pglfd.manif.util.get_manifold_dim_id`
%   method.
%
%   :param description: The manifold description, e.g., ``R3*SO3``.
%   :param dim_manifold: Dimension of the manifold - can be empty.
%   :param dim_tangent: Dimension of the tangent space - can be empty.
%   :return: A :mat:class:`pglfd.manif.manifold.ProductManifold`.

% get the submanifolds
if strcmp(description, 'SO2')
    manifold = pglfd.manif.manifold.SO2Manifold();
    return;
elseif strcmp(description, 'S2')
    manifold = pglfd.manif.manifold.S2Manifold();
    return;
elseif strcmp(description, 'SO3')
    manifold = pglfd.manif.manifold.SO3Manifold();
    return;
elseif strcmp(description, 'R3*SO3')
    manifolds = {pglfd.manif.manifold.EuclideanManifold(3) pglfd.manif.manifold.SO3Manifold()};
elseif strcmp(description, 'S2*R1*SO3')
    manifolds = {pglfd.manif.manifold.S2Manifold() pglfd.manif.manifold.EuclideanManifold(1) pglfd.manif.manifold.SO3Manifold()};
elseif strcmp(description, 'SO2*R2*SO3')
    manifolds = {pglfd.manif.manifold.SO2Manifold() pglfd.manif.manifold.EuclideanManifold(2) pglfd.manif.manifold.SO3Manifold()};
else
    manifolds = pglfd.manif.util.get_submanifolds(description);
end

% get the manifold dimension when inside Matlab
if coder.target('MATLAB')
    if ~exist('dim_manifold', 'var')
        [dim_manifold, dim_tangent, ~] = pglfd.manif.util.get_manifold_dim_id(description);
    end
end

% create the product manifold
manifold = pglfd.manif.manifold.ProductManifold(dim_manifold, dim_tangent, manifolds);
end
