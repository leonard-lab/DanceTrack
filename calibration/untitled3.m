clear all;
close all;

addpath('toolbox');

D = load('pts.dat');

pts_I = D(:, [1 2]);
pts_W = D(:, [3 4 5]);