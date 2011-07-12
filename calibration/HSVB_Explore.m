clear all;
close all;

%I = imread('../stills/calib.jpg');
%I = imread('../stills/Y13_1.jpg');
I = imread('../stills/CP1.bmp');
BG = imread('../stills/DanceBackground.bmp');

HSV = rgb2hsv(I);
H = HSV(:,:,1);
S = HSV(:,:,2);
V = HSV(:,:,3);
R = I(:, :, 1);
G = I(:, :, 2);
B = I(:, :, 3);

HSVB = rgb2hsv(BG);
HB = HSVB(:, :, 1);
SB = HSVB(:, :, 2);
VB = HSVB(:, :, 3);
RB = BG(:, :, 1);
GB = BG(:, :, 2);
BB = BG(:, :, 3);

hmin = 0.12;%40/360;
hmax = 0.25;%80/360;
smin = 0.15;
smax = 1;
vmin = 0.65;
vmax = 1;

mH = (H <= hmax).*(H >= hmin);
mS = (S <= smax).*(S >= smin);
mV = (V <= vmax).*(V >= vmin);
M = mH.*mS.*mV;

hskin_min = 6/360;
hskin_max = 30/360;
mSkin = (H <= hskin_max).*(H >= hskin_min);

mB = VB - V > 0.8;
BB = VB - V;

% [h, w] = size(M);
% dy = 20;
% win = 10;
% w1 = 0.5;  w2 = 0.5;
% 
% P = zeros(size(M));
% for x = 1 : w,
%     for y = (win + dy + 2) : h - (win + dy + 2),
%         ww = BB(y + dy + [1 : win], x) - BB(y - dy - [1 : win], x);
%         P(y,x) = w1*M(y,x) + w2*sum(ww)/(2*win);
%     end
% end
% 
% subplot(2,2,1)
% imagesc(P)
% colormap gray
% 
% mP = P > 0.3;
% subplot(2,2,2)
% imagesc(mP)
% colormap gray
% 
% mPs = medfilt2(mP, [3, 3]);
% subplot(2,2,3)
% imagesc(mPs)

subplot(2,2,1)
imshow(I)

MM = or(mB, M);
subplot(2,2,2)
imagesc(MM);
colormap gray

MMs = medfilt2(MM, [3, 3]);
subplot(2,2,3)
imagesc(MMs)

if 0,

figure
subplot(2,2,1)
imshow(H)
subplot(2,2,2)
imshow(S)
subplot(2,2,3)
imshow(V)
subplot(2,2,4)
imshow(I)

figure
subplot(2,2,1)
imshow(mH)
subplot(2,2,2)
imshow(mS)
subplot(2,2,3)
imshow(mV)
subplot(2,2,4)
imshow(M)

C = bwconncomp(M);
P = regionprops(C, {'Area', 'Centroid', 'EquivDiameter', 'BoundingBox'});

[~, order] = sort([P.Area], 2, 'descend');
P = P(order);

figure
image(I)
hold on

c = exp(i*linspace(0,2*pi));

MM = uint8(zeros(size(M)));
[h, w] = size(H);
pts_I = [];

for rx = 1 : 20,%length(P)
    x0 = P(rx).Centroid(1);
    y0 = P(rx).Centroid(2);
    r0 = P(rx).EquivDiameter/2;
    
    bb = P(rx).BoundingBox;
    
    plot(x0 + r0*real(c), y0 + r0*imag(c), 'r-', x0, y0, 'rx')
    rectangle('Position', bb, 'EdgeColor', 'g')
    
    px = [floor(bb(1)) : ceil(bb(1) + bb(3))];
    py = [floor(bb(2)) : ceil(bb(2) + bb(4))];
    py = py(py > 0);
    py = py(py <= h);
    px = px(px > 0);
    px = px(px <= w);
    
    MM(py, px) = 1;
    
    pts_I = [pts_I; x0 h-y0];
end

MMM = uint8(zeros(size(I)));
MMM(:,:,1) = MM;
MMM(:,:,2) = MM;
MMM(:,:,3) = MM;

Hat_H = H(MM > 0);
Hat_S = S(MM > 0);
Hat_V = V(MM > 0);
All_H = H(:);
All_S = S(:);
All_V = V(:);

b = linspace(0,1);

figure
subplot(3,2,1)
hist(Hat_H, b)
xlim([min(b) max(b)])
subplot(3,2,3)
hist(Hat_S, b)
xlim([min(b) max(b)])
subplot(3,2,5)
hist(Hat_V, b)
xlim([min(b) max(b)])

subplot(3,2,2)
hist(All_H, b)
xlim([min(b) max(b)])
subplot(3,2,4)
hist(All_S, b)
xlim([min(b) max(b)])
subplot(3,2,6)
hist(All_V, b)
xlim([min(b) max(b)])

ysplits = [0 150 225 290 325 350];

figure
plot(pts_I(:,1), pts_I(:, 2), 'x')
hold on
plot(repmat([100 600], [length(ysplits) 1])', repmat(ysplits', [1 2])', 'g-')

D = [];
for sx = 1 : 5,
    ix = find(and(pts_I(:, 2) > ysplits(sx), pts_I(:, 2) < ysplits(sx+1)));
    yy = pts_I(ix, 2);
    xx = pts_I(ix, 1);
    [xx, ord] = sort(xx);
    yy = yy(ord);
    
    for jx = 1 : length(xx),
        text(xx(jx), yy(jx), sprintf('(%d, %d)', jx-1, sx-1))
    end
    
    D = [D; xx h-yy [0 : 3]' (sx-1)*ones(4,1) zeros(4, 1)];
end

if 0,
save('pts.dat', 'D', '-ascii')
end

end