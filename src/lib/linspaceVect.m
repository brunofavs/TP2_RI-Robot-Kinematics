function MQ = linspaceVect(Qi, Qf, N)

%Inicializar MQ com o mesmo numero de juntas que Qi
MQ = zeros([size(Qi,1) N]);

for n=1:size(Qi,1)
    %Para cada junta interpolar entre Qi e Qf em N passos
    MQ(n,:) = linspace(Qi(n), Qf(n), N);
end