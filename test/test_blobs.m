clear all;  close all;

WRITE_IMAGE = 1;
RUN_TEST = 1;
READ_DATA = 1;
NOISE = 0;

I = imread('../stills/Blobs1.bmp');
I = rgb2gray(I);
M = uint8(zeros(size(I)));
M([64 : 110], [250 : 310]) = 1;
I = M.*I;

X = [267; 276; 290];
Y = [88; 92; 82];
A = 2*[1 1 1];
B = 2*[1 1 1];
Phi = [0 0 0];

num_ell = length(X);
Sinv = cell(num_ell, 1);
Ss = cell(num_ell, 1);
R = cell(num_ell, 1);
M = cell(num_ell, 1);
D = [];
P = [];

for ix = 1 : num_ell,
    sa2 = (A(ix))^2;
    sb2 = (B(ix))^2;
    p = Phi(ix);
    sp = sin(p);
    cp = cos(p);
    t = (sa2-sb2)*cp*sp;
        
    S = [sa2*cp*cp+sb2*sp*sp t; t sb2*cp*cp+sa2*sp*sp];
    Ss{ix} = S;
    Sinv{ix} = inv(S);
    R{ix} = [X(ix); Y(ix)];
    
    x = X(ix);
    y = Y(ix);
    xx = S(1,1);
    xy = S(1,2);
    yy = S(2,2);
    minor = B(ix);
    major = A(ix);
    o = Phi(ix);
    a = pi*minor*major;
    
    M{ix} = [x; y; xx; xy; yy; minor; major; a; o];
    
    D = [D; M{ix}'];
end
Do = D;

imagesc(I)
colormap gray

if WRITE_IMAGE,
    save('test_initials.dat', 'D', '-ASCII');
    imwrite(I, 'test_ellipse.bmp');
end

n_look = num_ell + (NOISE > 0)*1;
if RUN_TEST,
    system(sprintf('./testDSGYA_Segmenter %d', num_ell))
end

if READ_DATA,
   D = load('test_out.dat', '-ASCII');
   [n_found, ~] = size(D);
   Xf = D(:,1);
   Yf = D(:,2);
   XXf = D(:,3);
   XYf = D(:,4);
   YYf = D(:,5);
   mf = D(:,6);
   Mf = D(:,7);   
   Af = D(:,8);   
   Of = D(:,9);
   
   hold on
   c = exp(i*linspace(0, 2*pi));
   for ix = 1 : n_found,
       e = Xf(ix) + i*Yf(ix) + (Mf(ix)*real(c) + i*mf(ix)*imag(c))*exp(-i*Of(ix)*pi/180);
       plot(e, 'g-')
       e2 = M{ix}(1) + i*M{ix}(2) + (M{ix}(7)*real(c) + i*M{ix}(6)*imag(c))*exp(-i*M{ix}(9)*pi/180);
       plot(e2, 'r--')
   end
   axis xy
end