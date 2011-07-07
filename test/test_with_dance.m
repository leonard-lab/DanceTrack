clear all;
close all;

%I = imread('../stills/calib.jpg');
%n = 20;

I = imread('../stills/Y13_1.jpg');
n = 13;

HSV = rgb2hsv(I);
H = HSV(:,:,1);
S = HSV(:,:,2);
V = HSV(:,:,3);

hmin = 0.12;%40/360;
hmax = 0.2;%80/360;
smin = 0.15;
smax = 1;
vmin = 0.65;
vmax = 1;

mH = (H <= hmax).*(H >= hmin);
mS = (S <= smax).*(S >= smin);
mV = (V <= vmax).*(V >= vmin);
M = mH.*mS.*mV;
M([450 : end], :) = 0;

figure(1)
subplot(1,2,1)
imshow(M)
colormap gray

imwrite(M, 'test_ellipse.bmp');

system(sprintf('./testDSGYBlobber %d', n + 1))


D = load('test_out.dat', '-ASCII');
[n_found, ~] = size(D);
Xf = D(:,1);
Yf = D(:,2);
Of = D(:,3);
Af = D(:,4);
Mf = D(:,5);
mf = D(:,6);

subplot(1,2,2)
imshow(I)
hold on
c = exp(i*linspace(0, 2*pi));
for ix = 1 : n_found,
    e = Xf(ix) + i*Yf(ix) + (Mf(ix)*real(c) + i*mf(ix)*imag(c))*exp(-i*Of(ix)*pi/180);
    plot(e, 'g-')
end
