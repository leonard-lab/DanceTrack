clear all;
close all;

I = imread('../stills/DanceShort1_f1.jpg');
N = 13;

imshow(I)
hold on;

D = [];
for ix = 1 : N
    p = ginput(1);
    D = [D; p];
    plot(p(1), p(2), 'r+')
end

out_file = '../build/initials.dat';
save(out_file, 'D', '-ASCII');