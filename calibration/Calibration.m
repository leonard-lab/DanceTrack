clear all;
close all;

addpath('toolbox');

D = load('pts.dat');

pts_I = D(:, [1 2])';
%pts_I(2,:) = 480 - pts_I(2,:);
pts_W = D(:, [3 4 5])';

[~, N_p] = size(pts_I);

x_1 = pts_I;
X_1 = pts_W;

n_ima = 1;
nx = 640;
ny = 480;

init_intrinsic_param

[om_i, T_i, R_i] = compute_extrinsic_init(pts_I, pts_W, fc, cc, kc, alpha_c);
[om, T, R, JJ_kk] = compute_extrinsic_refine(om_i, T_i, pts_I, pts_W, fc, cc, kc, alpha_c, 20, inf);

WriteCalibrationData('DanceCal.dat', fc, cc, alpha_c, kc, R, T);

X = [];  Y = [];
for x = 0 : 3,
    for y = 0 : 4,
        rW = [x y 0]';
        rI = project_points(rW, om, T, fc(1), cc, kc);
        X = [X; rI(1)];
        Y = [Y; rI(2)];
    end
end

I = imread('../stills/calib.jpg');
imshow(I)
hold on
plot(X, Y, 'gx')