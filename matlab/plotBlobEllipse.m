function h = plotBlobEllipse(x, y, m, M, o, f)

if nargin < 6,
    f = 'r-';
end

c = exp(i*linspace(0, 2*pi));
e = x + i*y + (M*real(c) + i*m*imag(c))*exp(-i*o);
h = plot(e, f);