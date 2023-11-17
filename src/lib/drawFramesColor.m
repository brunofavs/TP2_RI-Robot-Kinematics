function H = drawFramesColor(AA, P, F, c)

if numel(c)<size(AA,3)+1
    error('Not enough colors')
end

T = eye(4);
H = zeros(size(AA,3),1);

patch('Faces', F, 'Vertices', P(1:3,:)', 'FaceColor', c(1));

for k=1:size(AA,3)
    T = T * AA(:,:,k);
    Pk = T*P;
    H(k) = patch('Faces', F, 'Vertices', Pk(1:3,:)', 'FaceColor', c(k+1));
end
