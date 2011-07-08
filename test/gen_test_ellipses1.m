clear all;  close all;

WRITE_IMAGE = 1;
READ_DATA = 1;
RUN_TEST = 1;
NOISE = 0.00;
PRIOR_NOISE = 50;
w = 320;
h = 240;

I = zeros(h, w);

thresh = 1;

% X = [200 200 50];
% Y = [100 100 135];
% A = 0.5*[100 100 20];
% B = 0.5*[20 20 100];
% PHI = [60 0 45]*pi/180;

n = 4;
b = 20;
a = 50;
b = 20; 
X = b + (w - 2*b)*rand(1, n);
Y = b + (h - 2*b)*rand(1, n);
amin = 5;  amax = 10;
bmin = 20;  bmax = 40;
A = amin + (amax-amin)*rand(1,n);
B = bmin + (bmax-bmin)*rand(1,n);
PHI = 2*pi*randn(1, n);

num_ell = length(X);
Sinv = cell(num_ell, 1);
R = cell(num_ell, 1);
M = cell(num_ell, 1);
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
    xm = X(ix) + sqrt(PRIOR_NOISE)*randn(1);
    ym = Y(ix) + sqrt(PRIOR_NOISE)*randn(1);
    xxm = S(1,1) + PRIOR_NOISE*randn(1);
    xym = S(1,2) + PRIOR_NOISE*randn(1);
    yym = S(2,2) + PRIOR_NOISE*randn(1);
    
    SS =[xxm xym; xym yym];
    [v,d] = eig(SS);
    
    minor = sqrt(d(1,1));
    major = sqrt(d(2,2));
    o = -atan2(v(2,2),v(2,1))*180/pi;
    a = pi*minor*major;
    
    M{ix} = [xm; ym; xxm; xym; yym; minor; major; a; o];
    
    D = [D; M{ix}'];
end
Do = D;

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

%I([5 : 10], [5 : 10]) = 1;
%I([15 : 20], [15 : 20]) = 1;

imagesc(I)
colormap gray

if WRITE_IMAGE,
    save('test_initials.dat', 'D', '-ASCII');
    imwrite(I, 'test_ellipse.bmp');
end

n_look = num_ell + (NOISE > 0)*1;
if RUN_TEST,
    system(sprintf('./testDSGYA_Segmenter %d 1', num_ell))
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