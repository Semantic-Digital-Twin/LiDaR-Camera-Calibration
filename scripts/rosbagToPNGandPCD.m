% rosbag to .png and .pcd
baggins = '/home/art5gpc6/datasets/ARTGarage/checkerboard_li32_cam_03.bag';
matdir = '/home/art5gpc6/matlab/data';
bag = rosbag(baggins);
imageBag = select(bag,'Topic','/camera/camera/color/image_raw');
pc2Bag = select(bag,'Topic','/velodyne_points');
imageMsgs = readMessages(imageBag);
pc2Msgs = readMessages(pc2Bag);
ts1 = timeseries(imageBag);
ts2 = timeseries(pc2Bag);
t1 = ts1.Time;
t2 = ts2.Time;

k = 1;
if size(t2,1) > size(t1,1)
    for i = 1:size(t1,1)
        [val,indx] = min(abs(t1(i) - t2));
        if val <= 0.1
            idx(k,:) = [i indx];
            k = k + 1;
        end
    end
else
    for i = 1:size(t2,1)
        [val,indx] = min(abs(t2(i) - t1));
        if val <= 0.1
            idx(k,:) = [indx i];
            k = k + 1;
        end
    end
end

pc2FilesPath = fullfile(matdir,'PointCloud2s');
imageFilesPath = fullfile(matdir,'Images');
if ~exist(imageFilesPath,'dir')
    mkdir(imageFilesPath);
end
if ~exist(pc2FilesPath,'dir')
    mkdir(pc2FilesPath);
end

for i = 1:length(idx)
    I = readImage(imageMsgs{idx(i,1)});
    pc = pointCloud(readXYZ(pc2Msgs{idx(i,2)}));
    n_strPadded = sprintf('%04d',i) ;
    pc2FileName = strcat(pc2FilesPath,'/',n_strPadded,'.pcd');
    imageFileName = strcat(imageFilesPath,'/',n_strPadded,'.png');
    imwrite(I,imageFileName);
    pcwrite(pc,pc2FileName);
end
