clc;
clear;
pre_frame = 101;
idx = pre_frame + 1;
real=load('00.txt');
delta_real = compute_delta_matrix(real(idx + 1,:),real(idx,:));


filename = sprintf('../result/couple-%d-%d', pre_frame, pre_frame + 1);
fid = fopen(filename);
fread(fid, 2, 'int32');
fread(fid, 16, 'double');
fsize = fread(fid, 1, 'int32');
features = [];
for index = 1:fsize
    feature.u1p = fread(fid, 1, 'float32');
    feature.v1p = fread(fid, 1, 'float32');
    feature.i1p = fread(fid, 1, 'int32');
    fread(fid, 36, 'int8');
    features = [features feature];
end
    
    
couples = [];
for i = 1:300
    filename = sprintf('../matrix_result/delta_%d', i - 1);
    fid = fopen(filename);
    couple.pid = fread(fid, 1, 'int32');
    couple.cid = fread(fid, 1, 'int32');
    couple.p1 = fread(fid, 1, 'int32');
    couple.p2 = fread(fid, 1, 'int32');
    couple.p3 = fread(fid, 1, 'int32');
    m=zeros(4,4);
    for j=1:4
        for k=1:4
            v=fread(fid,1,'double');
            m(j,k)=v;
        end
    end
    couple.matrix = m;
    couples = [couples couple];
end
trans_deltas = [];
rotation_deltas = [];
for i = 1:300
    trans_delta = sum((delta_real(1:3,4) - couples(i).matrix(1:3,4)).^2);
    if trans_delta > 0
        trans_deltas = [trans_deltas trans_delta];
    end
    rotation_delta = (asin(delta_real(1, 3)) -... 
                        asin(couples(i).matrix(1,3)))^2;
    rotation_deltas = [rotation_deltas rotation_delta];
end


function delta = compute_delta_matrix(C,P)
    C = [reshape(C,4,3)';[0 0 0 1]];
    P = [reshape(P,4,3)';[0 0 0 1]];
    delta = C / P;
end