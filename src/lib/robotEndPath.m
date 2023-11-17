function [X, Y, Z] = robotEndPath(AAA)

P = zeros(3, size(AAA,4));

for n=1:size(AAA,4)
    A = eye(4);

    for j = 1:size(AAA,3)
        A = A * AAA(:,:,j, n);
    end

    P(:, n) = A(1:3,4);
end

X = P(1,:);
Y = P(2,:);
Z = P(3,:);