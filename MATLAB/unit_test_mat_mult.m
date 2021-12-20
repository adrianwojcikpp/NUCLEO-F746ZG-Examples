%% Matrix multiplication unit test

%%% File info 
%
% ************************************************************************
%
%  @file     unit_test_mat_mult.m
%  @author   Adrian Wojcik
%  @version  1.0
%  @date     15-Dec-2020 08:55:55
%  @brief    TODO
%
% ************************************************************************
%
close all; clc;
clear A x y
%% Matrix multiplication

A = [1 2 3; 2 3 1; 1 3 2];
x = [0.1 0.2 0.3]';

yref = A*x;

disp("LHS ARGUMENT (MATRIX)");
disp(A);

disp("RHS ARGUMENT (COLUMN VECTOR)");
disp(x);

disp("REFERENCE RESULT:");
disp(yref);

%% Display CMSIS data
fprintf("A MATRIX:\n");
A = MAT2CMSIS(A);
fprintf("\n");

fprintf("X VECTOR:\n");
x = MAT2CMSIS(x);
fprintf("\n");

fprintf("Y VECTOR:\n");
y = MAT2CMSIS([ 0 0 0 ]);
fprintf("\n");

fprintf("YREF VECTOR:\n");
yref = MAT2CMSIS(yref);
fprintf("\n");
