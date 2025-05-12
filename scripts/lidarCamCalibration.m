%% LiDaR Camera Calibration
matdir = '/home/art5gpc6/matlab/data';
imageFilesPath = fullfile(matdir,'Images');
pc2FilesPath = fullfile(matdir,'PointCloud2s');
imageFolder = imageFilesPath; % Folder with camera images (.png)
pc2Folder = pc2FilesPath;     % Folder with lidar point clouds (.pcd)
squareSize = 56;

% cameraParams should be a cameraParameters object loaded from your calibration

% List and sort image and point cloud files to ensure correct pairing
imageFiles = dir(fullfile(imageFolder, '*.png'));
pc2Files = dir(fullfile(pc2Folder, '*.pcd'));

% Sort files by name to align pairs
[~, idxImg] = sort({imageFiles.name});
imageFiles = imageFiles(idxImg);

[~, idxPC] = sort({pc2Files.name});
pc2Files = pc2Files(idxPC);

assert(numel(imageFiles) == numel(pc2Files), ...
    'Number of images and point clouds must be equal.');

numFrames = numel(imageFiles);

% Full file paths
imageFileNames = fullfile({imageFiles.folder}, {imageFiles.name});
pc2FileNames = fullfile({pc2Files.folder}, {pc2Files.name});

%% Step 1: Extract 3-D checkerboard corners from camera images (in camera coordinates)

% estimateCheckerboardCorners3d returns:
% - imageCorners3d: 3D checkerboard corners in camera coordinate system (Nx3xM)
% - boardSize: checkerboard pattern size (number of inner corners)
% - dataUsed: logical vector indicating successful detection per frame

[imageCorners3d, boardSize, dataUsed] = estimateCheckerboardCorners3d(imageFileNames, cameraParams, squareSize);

% Keep only frames with successful checkerboard detection
% imageCorners3d = imageCorners3d(:,:,dataUsed);
imageFileNames = imageFileNames(dataUsed);
pc2FileNames = pc2FileNames(dataUsed);

%% Step 2: Extract checkerboard planes from lidar point clouds

% detectRectangularPlanePoints returns:
% - lidarPlanes: cell array of checkerboard plane points for each frame
% - framesUsed: logical vector indicating successful plane detection
% - indices: indices of lidar points belonging to checkerboard plane

[lidarPlanes, framesUsed, indices] = detectRectangularPlanePoints(pc2FileNames, boardSize, 'RemoveGround', true);

% Keep only frames with successful lidar plane detection
imageCorners3d = imageCorners3d(:,:,framesUsed);
imageFileNames = imageFileNames(framesUsed);
pc2FileNames = pc2FileNames(framesUsed);

%% Step 3: Estimate the rigid transformation (extrinsics) between lidar and camera

% estimateLidarCameraTransform returns:
% - tform: rigid3d object representing lidar-to-camera transform
% - errors: struct with calibration error metrics

[tform, errors] = estimateLidarCameraTransform(lidarPlanes, imageCorners3d, cameraParams);

% Display estimated rotation and translation
disp('Estimated Rotation Matrix (lidar to camera):');
disp(tform.Rotation);

disp('Estimated Translation Vector (lidar to camera):');
disp(tform.Translation);


%% LiDaR projection
%% Step 4: Project lidar points onto camera images to verify calibration

% Select a frame to visualize
frameIdx = 1;

% Read point cloud and image for this frame
ptCloud = pcread(pc2FileNames{frameIdx});
I = imread(imageFileNames{frameIdx});

% Project lidar points onto the image
[imagePoints, validIndices] = projectLidarPointsOnImage(ptCloud, cameraParams, tform);

% Visualise projection
figure;
imshow(I);
hold on;
plot(imagePoints(:,1), imagePoints(:,2), 'r.', 'MarkerSize', 5);
title('Lidar Points Projected onto Camera Image');
hold off;

%% Optional Step 5: Fuse camera colors onto lidar point cloud (if desired)

% Fuse camera colors to the lidar point cloud using the estimated transform
ptCloudColored = fuseCameraToLidar(I, ptCloud, cameraParams, tform);

% Visualize colored point cloud
figure;
pcshow(ptCloudColored);
title('Colored Point Cloud Fused from Camera Image');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
