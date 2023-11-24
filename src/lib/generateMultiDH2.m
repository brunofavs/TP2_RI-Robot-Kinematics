function MDH = generateMultiDH2(DH, MQ, t)

if nargin<3
    %Se não tiverem sido fornecidos os 3 argumentos vou definir o t com um
    %default value
    t = zeros(size(DH,1));
end

%Definir MDH como repetições de DH
MDH = repmat(DH, [1 1 size(MQ,2)]);
% MDH = zeros(size(DH, 1), 4, size(MQ, 2));

for n = 1:size(MQ,2)
    for j=1:size(DH,1)
        if t(j)==1
            %A junta j é prismatica, incrementar d
            MDH(j,3,n) = MDH(j,3,n) + MQ(j,n);
        else
            %A junta j é rotacional, incrementar theta
            MDH(j,1,n) = MDH(j,1,n) + MQ(j,n);
        end

        if ~isreal(MDH)
            disp("MDH COMPLEX")
        end

        
    end
end