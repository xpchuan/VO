for i = 1:900
    filename = sprintf('../result/couple-%d-%d', i-1, i);
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
    for index = 1:fsize
        count = fread(fid, 1, 'int32');
        if count < 0
            features(index).final = 1;
            features(index).count =  -count;
        else 
            features(index).final = 0;
            features(index).count = count;
        end     
    end
end
save features.mat features
