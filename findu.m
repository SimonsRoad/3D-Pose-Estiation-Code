% use SVD to find vector u of the plane normal
function u = findu(objectVectors)
    A = objectVectors;
    [U,S,V] = svd(A);
    s = svd(A);

    m = min(s);

    index = find(s==m);
    l = length(index);
    u = V(:,index(l))';
end