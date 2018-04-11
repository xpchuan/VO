%%
% 0115_31--ok
date = '0118_16';

clear temp 
temp = textread(['./CANdata/UTM_N_',date,'.txt']);
UTM_N_group(:,1) = temp(2:2:end);
UTM_N_group(:,2) = temp(1:2:end);

clear temp
temp = textread(['./CANdata/UTM_E_',date,'.txt']);
UTM_E_group(:,1) = temp(2:2:end);
UTM_E_group(:,2) = temp(1:2:end);

temp = textread(['./CANdata/speed_RL_',date,'.txt']);
speed_RL_group(:,1) = temp(2:2:end);
speed_RL_group(:,2) = temp(1:2:end) - temp(1);

temp = textread(['./CANdata/speed_RR_',date,'.txt']);
speed_RR_group(:,1) = temp(2:2:end);
speed_RR_group(:,2) = -temp(1:2:end) + temp(1);

temp = textread(['./CANdata/yaw_',date,'.txt']);
yaw_group(:,1) = temp(2:2:end) - temp(2);
yaw_group(:,2) = temp(1:2:end) - temp(1);

%%
MaxIter = length(UTM_N_group);

for i=1:MaxIter
    Length_UTM(i,1) = UTM_N_group(i,1);
    Length_UTM(i,2) = sqrt(UTM_N_group(i,2)^2 + UTM_E_group(i,2)^2);
end

sqrt(UTM_N_group(MaxIter,2)^2 + UTM_E_group(MaxIter,2)^2)

% figure % plot N
% plot(UTM_N_group(1:MaxIter,1),UTM_N_group(1:MaxIter,2),'b.');hold on;
% % ylim([-30,30])
% 
% figure % plot E
% plot(UTM_E_group(1:MaxIter,1),UTM_E_group(1:MaxIter,2),'b.');hold on;
% % ylim([-5,5])

% figure % plot trace
plot(UTM_N_group(1:MaxIter,2),UTM_E_group(1:MaxIter,2),'b.');hold on;
%[arclenGt1,seglenGt1] = arclength(UTM_N_group(1:MaxIter,2),UTM_E_group(1:MaxIter,2),'linear'); 

% xlim([-30,30])
axis square
daspect([1 1 1]);