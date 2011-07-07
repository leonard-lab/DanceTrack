clear all; close all;
% biograph viewer doesn't close automatically C
child_handles = allchild(0);
names = get(child_handles,'Name');
k = find(strncmp('Biograph Viewer', names, 15));
close(child_handles(k))

rows = 5;
cols = 5;

ep = 5/(rows*cols);

Ntest = 1;
DO_DRAW = 0;

for tx = 1 : Ntest,
    A = double(rand(rows, cols) > (1-ep));
    
    G = sparse([zeros(rows, rows) A; A' zeros(cols, cols)]);
    
    [S, C] = graphconncomp(G);
    
    if(DO_DRAW)
        h = view(biograph(G));
        colors = jet(S);
        for i = 1 : numel(h.nodes),
            h.Nodes(i).Color = colors(C(i), :);
        end
    end
    
    L = A;
    for ix = 1 : rows,
        for jx = 1 : cols,
            if A(ix,jx),
                L(ix, jx) = C(ix);
            end
        end
    end
    Lo = L;
    
    %e = find(L);
    %v = L(e);
    %vu = unique(v);
    
    %for kx = 1 : numel(e),
    %    L(e(kx)) = find(vu == v(kx));
    %end
    
    D = reshape(A', [1 numel(A)]);
    save('Adj.dat', 'D', '-ASCII')
    
    system(sprintf('./testBiCC %d %d', rows, cols));
    
    Di = load('BiCCout.dat', '-ASCII');
    Lb = zeros(cols, rows);
    Lb([1 : numel(Di)]) = Di;
    Lb = Lb';
    
    if(max(max(L - Lb)))
        disp('ERROR!')
        pause
    end
end