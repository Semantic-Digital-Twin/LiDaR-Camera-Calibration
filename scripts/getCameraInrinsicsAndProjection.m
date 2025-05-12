% Camera Intrinsics
matdir = '/home/art5gpc6/matlab/data';
imageFilesPath = fullfile(matdir,'Images');
% Step 1: Load all calibration images (PNG files)
imageFilePattern = fullfile(imageFilesPath, '*.png');
imds = imageDatastore(imageFilePattern);
imageFileNames = imds.Files;

% Step 2: Detect checkerboard points, exclude partial detections automatically
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames, 'PartialDetections', false);

% Filter image file names to keep only those with full checkerboard detection
validImageFileNames = imageFileNames(imagesUsed);

% Step 3: Generate world coordinates of checkerboard corners
squareSize = 56; % in millimeters, adjust to your checkerboard square size
worldPoints = patternWorldPoints("checkerboard", boardSize, squareSize);

% Step 4: Read one valid image to get image size
I = imread(validImageFileNames{1});
imageSize = [size(I, 1), size(I, 2)];

% Step 5: Calibrate the camera using only valid images
[cameraParams, imagesUsedFinal, estimationErrors] = estimateCameraParameters( ...
    imagePoints, worldPoints, 'ImageSize', imageSize);

% (Optional) Projection and visualise calibration results
% Step 6: (Optional) visualize calibration results
figure; showReprojectionErrors(cameraParams);
figure; showExtrinsics(cameraParams);
drawnow;
% Step 6: (Optional) Projection of nth frame
n = 600;
figure; imshow(validImageFileNames{n}); 
hold on;
plot(imagePoints(:,1,n), imagePoints(:,2,n),'go');
plot(cameraParams.ReprojectedPoints(:,1,n),cameraParams.ReprojectedPoints(:,2,n),'r+');
legend('Detected Points','ReprojectedPoints');
hold off;
