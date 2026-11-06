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

function [q] = rotmat_to_quaternion(rotm)
%ROTMAT_TO_QUATERNION Convert a rotation matrix to a quaternion.
%   The conversion uses the Chiaverini-Siciliano method.
%
%   :param rotm: The rotation matrix to convert.
%   :return: The quaternion

q = 0.5 * [ ...
    sqrt(abs(1+rotm(1, 1)-rotm(2, 2)-rotm(3, 3))) * pglfd.util.sign(rotm(3, 2)-rotm(2, 3)); ...
    sqrt(abs(1-rotm(1, 1)+rotm(2, 2)-rotm(3, 3))) * pglfd.util.sign(rotm(1, 3)-rotm(3, 1)); ...
    sqrt(abs(1-rotm(1, 1)-rotm(2, 2)+rotm(3, 3))) * pglfd.util.sign(rotm(2, 1)-rotm(1, 2)); ...
    sqrt(abs(1+rotm(1, 1)+rotm(2, 2)+rotm(3, 3))) ...
    ];
end
