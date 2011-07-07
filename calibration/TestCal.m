clear all;  close all;

[f, c, alpha, k, R, T, lines] = ReadCalibrationData('DanceCal.dat');

X = [];  Y = [];
for x = 0 : 3,
    for y = 0 : 4,
        rW = [x y 0]';
        rI = worldToImage(rW, f, c, R, T, 0);
        X = [X; rI(1)];
        Y = [Y; rI(2)];
    end
end

I = imread('../stills/calib.jpg');
imshow(I)
hold on
plot(X, Y, 'gx')