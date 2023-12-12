function AAc = accTlinks(DH)

    addpath('./lib')

AAc = zeros(4,4,size(DH,1));


AAc(:,:,1) = tlink(DH(1,1),DH(1,2),DH(1,3),DH(1,4));

for d=2:size(DH,1)

    AAc(:,:,d) = AAc(:,:,d-1)*tlink(DH(d,1),DH(d,2),DH(d,3),DH(d,4));
end