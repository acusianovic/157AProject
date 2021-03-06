% aerobee comparison
load('aeroBeeRef.mat','aeroBee')
load('Aerobee150ADragData.mat','Aerobee150ADragData')

v = 21:10:100000;
a = 294.9*3.28;
M = v./a;
Cd = zeros(length(v),1);
CdB = zeros(length(v),1);
CdF = zeros(length(v),1);
CdP = zeros(length(v),1);
Cde = zeros(length(v),1);
CdBase = zeros(length(v),1);
delCdT = zeros(length(v),1);
delCds = zeros(length(v),1);


for j = 1:length(v)
    [Cd(j),CdB(j),CdF(j),CdP(j),Cde(j),CdBase(j),delCdT(j),delCds(j)] = getDrag2(aeroBee,10000,v(j),a);  
end

figure
hold on
plot(M,Cd,M,CdB,M,CdF,M,CdP,M,Cde,M,CdBase,M,delCdT,M,delCds,'LineWidth',2)
plot(Aerobee150ADragData(1,:),Aerobee150ADragData(2,:),'r','LineWidth',2)
grid on
xlim([0.11 7])
xlabel('Mach Number')
ylabel('Drag Coefficient')
legend('Cd','CdB','CdF','Cdp','Cde','CdBase','delCdT','delCds','Aerobee')