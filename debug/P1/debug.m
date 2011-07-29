clear all;  close all;

min_perim = 1;
min_area = 5;
max_area = 1000;
overlap = 2.0;
D = [min_perim min_area max_area overlap];
save('debug_params.dat', 'D', '-ASCII')

addpath('../../matlab');

I = imread('BlobsP1.bmp');
I = rgb2gray(I);
M = uint8(zeros(size(I)));
M([50 : 160], [140 : 200]) = 1;
I = M.*I;

imagesc(I)
colormap gray

[X, Y, XX, XY, YY, m, M, A, O] = readDSGYABlobs('blobs-in.dat');
N = length(X);

%IX = [1 : N];
IX = [2 3 4 5];
X_i = X(IX);
Y_i = Y(IX);
XX_i = XX(IX);
XY_i = XY(IX);
YY_i = YY(IX);
m_i = m(IX);
M_i = M(IX);
A_i = A(IX);
O_i = O(IX);

writeDSGYABlobs('debug_in.dat', X_i, Y_i, XX_i, XY_i, YY_i, m_i, M_i, A_i, O_i);

N = length(X_i);
hold on
for ix = 1 : N,
    plotBlobEllipse(X_i(ix), Y_i(ix), m_i(ix), M_i(ix), O_i(ix), 'g-');
    %text(X_i(ix), Y_i(ix), num2str(ix));
end

imwrite(I, 'debug.bmp');
eval(['!./debug ' num2str(N)])

[X_o, Y_o, XX_o, XY_o, YY_o, m_o, M_o, A_o, O_o] = readDSGYABlobs('debug_out.dat');

for ix = 1 : N,
    plotBlobEllipse(X_o(ix), Y_o(ix), m_o(ix), M_o(ix), O_o(ix), 'r-');
end