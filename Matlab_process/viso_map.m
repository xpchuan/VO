clear;
clc;
fid=fopen('./pose_result.bin');
P=[];
for i=1:900
    m=zeros(4,4);
    for j=1:4
        for k=1:4
            v=fread(fid,1,'double');
            m(j,k)=v;
        end
    end
    P = [P; reshape(m(1:3,:)',1,3*4)];
end
fclose(fid);

fid2=fopen('./no_map_pose_result.bin');
NP=[];
for i=1:900
    m=zeros(4,4);
    for j=1:4
        for k=1:4
            v=fread(fid,1,'double');
            m(j,k)=v;
        end
    end
    NP = [NP; reshape(m(1:3,:)',1,3*4)];
end
fclose(fid2);


mend = 900;

plot(P(1:mend,4)-P(1,4),P(1:mend,12)-P(1,12), '.')
hold on
plot(NP(1:mend,4)-NP(1,4),NP(1:mend,12)-NP(1,12), '.')
hold on 
real=load('00.txt');
plot(real(1:mend,4)-real(1,4),real(1:mend,12)-real(1,12), '.')

legend('Map', 'No Map', 'GT')

% error = sqrt((real(mend,4) - P(mend,4))^2 + (real(mend,12) - P(mend,12))^2);
% 
% trans_errors = [];
% rotation_errors = [];
% for i = 2:mend
%     delta_P = compute_delta_matrix(P(i,:),P(i-1,:));
%     delta_real = compute_delta_matrix(real(i,:),real(i-1,:));
%     rotation_error = abs(abs(asin(delta_real(3,1))) - abs(asin(delta_P(3,1))));
%     trans_error = abs(sqrt(delta_P(1,4)^2 + delta_P(2,4)^2 + delta_P(3,4)^2)...
%                     - sqrt(delta_real(1,4)^2 + delta_real(2,4)^2 + delta_real(3,4)^2));
%     trans_errors = [trans_errors trans_error];
%     rotation_errors = [rotation_errors rotation_error];
% end
% 
% figure(2)
% plot(trans_errors)
% trans_error_sum = mean(trans_errors)
% rotation_errors_sum = sum(rotation_errors)
% 
% function delta = compute_delta_matrix(C,P)
%     C = [reshape(C,4,3)';[0 0 0 1]];
%     P = [reshape(P,4,3)';[0 0 0 1]];
%     delta = C / P;
% end
