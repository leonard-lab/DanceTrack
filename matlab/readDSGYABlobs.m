function [X, Y, XX, XY, YY, m, M, A, O] = readDSGYABlobs(filename)

D = load(filename, 'ASCII');
X = D(:,1);
Y = D(:,2);
XX = D(:,3);
XY = D(:,4);
YY = D(:,5);
m = D(:,6);
M = D(:,7);
A = D(:,8);
O = D(:,9);