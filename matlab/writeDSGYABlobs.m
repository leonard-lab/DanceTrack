function writeDSGYABlobs(filename, X, Y, XX, XY, YY, m, M, A, O)

D = [X Y XX XY YY m M A O];
save(filename, 'D', '-ASCII');