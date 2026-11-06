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

function [qmult] = quaternion_multiply(q1, q2)
%QUATERNION_MULTIPLY Product of q1 and q2.
%
%   :param q1: The first quaternion.
%   :param q2: The second quaternion.
%   :return: The quaternion product.

qmult = [q1(4) * q2(1) + q1(1) * q2(4) + q1(2) * q2(3) - q1(3) * q2(2) ...
    q1(4) * q2(2) + q1(2) * q2(4) + q1(3) * q2(1) - q1(1) * q2(3) ...
    q1(4) * q2(3) + q1(3) * q2(4) + q1(1) * q2(2) - q1(2) * q2(1) ...
    q1(4) * q2(4) - q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3)]';

end
