function [A,B,C,D] = ibvs_cstate()
% state equations
A = zeros(8,8);
B = zeros(8,14);
C = eye(8);
D = zeros(8,14);