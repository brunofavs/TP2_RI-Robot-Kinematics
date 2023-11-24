function AAA = calculateRobotMotion(MDH)

%4x4 x Number of links x number of steps
AAA = zeros(4, 4, size(MDH,1), size(MDH,3));

for k=1:size(MDH, 3)
    AAA(:, :, :, k) = tlinks(MDH(:,:, k));


    % if ~isreal(AAA)
    %     disp("COMPLEX")
    % end

    %  AAA(:, :, :, k) = tlinks(MDH(:,:, k));

end