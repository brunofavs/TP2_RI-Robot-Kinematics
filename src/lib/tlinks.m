function AA = tlinks(DH)

AA = zeros(4,4,size(DH,1));

for d=1:size(DH,1)
    AA(:,:,d) = tlink(DH(d,1),DH(d,2),DH(d,3),DH(d,4));
end