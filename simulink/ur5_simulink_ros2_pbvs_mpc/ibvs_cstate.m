function [A,B,C,D] = ibvs_cstate()
% state equations
A = zeros(6,6);
B = zeros(6,12);
C = eye(6);
D = zeros(6,12);