clc;
clear;
xd = 10;
yd = 10;

video = VideoWriter('features_correct.avi');
video.FrameRate = 7;
open(video);

for fi = 1:900
    
    fprintf('current frame: %d\n', fi);
    %path
    filename = sprintf('../result/couple-%d-%d', fi-1, fi);
    fid = fopen(filename);
    im_fname = sprintf('../data/image_0/%06d.png', fi-1);
    pic = imread(im_fname);
    
    %pre_read
    fread(fid, 2, 'int32');
    fread(fid, 16, 'double');
    fsize = fread(fid, 1, 'int32');
    
    %features fill
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
    
    %bucketing
    [rl, cl] = size(pic);
    bucket = zeros(ceil(rl / yd), ceil(cl / xd));
    for index = 1:fsize
        if features(index).final == 0
            bucket(ceil(features(index).v1p / yd),...
                    ceil(features(index).u1p / xd)) = bucket(ceil(features(index).v1p / yd),...
                    ceil(features(index).u1p / xd)) + 1;
        end
    end
    
    max_bucket = max(max(bucket));
    pic_out = uint8(zeros(rl, cl, 3));
    for r = 1:rl
        for c =1:cl 
            bucket_r = ceil(r / yd);
            bucket_c = ceil(c / xd);
            pic_out(r, c, 1) = pic(r, c);
            pic_out(r, c, 2) = int8(bucket(bucket_r, bucket_c)  / max_bucket * 255) ;
            pic_out(r, c, 3) = 0;
        end
    end
    writeVideo(video, pic_out)
    fclose(fid);
end
close(video);