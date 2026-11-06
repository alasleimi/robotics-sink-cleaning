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

% get some manifold values
man_cyl = pglfd.manif.util.get_productmanifold('SO2*R2*SO3');
tangent = [0.1 0.1 0.1 0.1 0.1 0.1]';
base = man_cyl.id_element;
point = man_cyl.exp(tangent, base);
input = eye(6);
g = [1 0 0 1 0.707 0.707 0 0]';
h = man_cyl.action(base, point, g);

% get some S2*R1*SO3 manifold values
man_sp = pglfd.manif.util.get_productmanifold('S2*R1*SO3');
tangent_s2r1 = [0.1 0.1 0.1 0.1 0.1 0.1]';
base_s2r1 = man_sp.id_element;
point_s2r1 = man_sp.exp(tangent_s2r1, base_s2r1);
input_s2r1 = eye(6);
g_s2r1 = [1 0 0 1 0.707 0.707 0 0]';
h_s2r1 = man_sp.action(base_s2r1, point_s2r1, g_s2r1);

% get values for task parametrization
man_cart = pglfd.manif.util.get_productmanifold('R3*SO3');
mu = man_cart.id_element;
covariance = eye(6);
A = eye(6);
b = man_cart.id_element;

% let the simulation run
sim('test_pgmanif');
rtwbuild('test_pgmanif', 'generateCodeOnly', 1);
% TODO evaluate results
