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

function [submanifolds] = get_submanifolds(description)
%GET_SUBMANIFOLDS Get submanifolds from a description
%   This method returns a cell array containing submanfolds described by
%   e.g. ``R3*SO3``.
%
%   :param description: The manifold description.
%   :return: A list of submanifolds.

len = 0;
for chr = description
    if chr == '*'
        len = len + 1;
    end
end
submanifolds = cell(1, len);
i = 1;
for manifold = strsplit(description, '*')
    if strfind(manifold{1}, 'R')
        % parse number
        num = str2double(manifold{1}(2:end));

        submanifolds{i} = pglfd.manif.manifold.EuclideanManifold(num);
    elseif strfind(manifold{1}, 'SO3')
        submanifolds{i} = pglfd.manif.manifold.SO3Manifold();
    elseif strfind(manifold{1}, 'S2')
        submanifolds{i} = pglfd.manif.manifold.S2Manifold();
    elseif strfind(manifold{1}, 'SO2')
        submanifolds{i} = pglfd.manif.manifold.SO2Manifold();
    else
        error(['Unknown Manifold ' manifold]);
    end
    i = i + 1;
end
end
