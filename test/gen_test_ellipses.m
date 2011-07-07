clear all;  close all;

WRITE_IMAGE = 1;
READ_DATA = 1;
RUN_TEST = 1;
NOISE = 0.00;
w = 320;
h = 240;

I = zeros(h, w);

thresh = 1;

n = 2; 
X = [200 200];
Y = [100 100];
A = 0.5*[100 100];
B = 0.5*[20 20];
PHI = [90 0]*pi/180;

% n = 3;
% b = 20;
% a = 50;
% b = 20;
% X = b + (w - 2*b)*rand(1, n);
% Y = b + (h - 2*b)*rand(1, n);
% A = (a-5)*randn(1, n) + 5;
% B = (b-5)*randn(1, n) + 5;
% PHI = 2*pi*randn(1, n);

num_ell = length(X);
Sinv = cell(num_ell, 1);
R = cell(num_ell, 1);
D = [];

for ix = 1 : num_ell,
    sa2 = (A(ix))^2;
    sb2 = (B(ix))^2;
    p = PHI(ix);
    sp = sin(p);
    cp = cos(p);
    t = (sa2-sb2)*cp*sp;
        
    S = [sa2*cp*cp+sb2*sp*sp t; t sb2*cp*cp+sa2*sp*sp];
    Sinv{ix} = inv(S);
    R{ix} = [X(ix); Y(ix)];
    D = [D; X(ix) Y(ix) S(1,1) S(1,2) S(2,2)];
end

for x = 1 : w,
    for y = 1 : h,
        for ix = 1 : num_ell;
            dr = [R{ix}(1) - x; R{ix}(2) - y];
            p = (dr'*Sinv{ix}*dr);
            if p < thresh,
                I(y,x) = 1;
            end
            if rand(1) < NOISE,
                I(y,x) = 1 - I(y,x);
            end
        end
    end
end

I([5 : 10], [5 : 10]) = 1;
I([15 : 20], [15 : 20]) = 1;

imagesc(I)
colormap gray

if WRITE_IMAGE,
    save('test_initials.dat', 'D', '-ASCII');
    imwrite(I, 'test_ellipse.bmp');
end

n_look = num_ell + (NOISE > 0)*1;
if RUN_TEST,
    system(sprintf('./testDSGYBlobberWHist %d %d', n_look, num_ell))
end

if READ_DATA,
   D = load('test_out.dat', '-ASCII');
   [n_found, ~] = size(D);
   Xf = D(:,1);
   Yf = D(:,2);
   Of = D(:,3);
   Af = D(:,4);
   Mf = D(:,5);
   mf = D(:,6);
   
   hold on
   c = exp(i*linspace(0, 2*pi));
   for ix = 1 : n_found,
       e = Xf(ix) + i*Yf(ix) + (Mf(ix)*real(c) + i*mf(ix)*imag(c))*exp(-i*Of(ix)*pi/180);
       plot(e, 'g-')
   end
end