%% stereo_single_measure.m
% Use existing stereoParams to read a stereo image pair for depth and checkerboard square measurement

clear; clc; close all;

%% 1. Load stereo calibration results
% Assume you have saved calibration results from MATLAB App as stereoParams.mat
% Or you already have stereoParams in workspace
load('stereoparams315_1.mat');  % Contains stereoParams struct

% View structure content
disp(stereoParams);

%% 2. Read a pair of test images
% Modify to your actual image path
leftImageFile = 'C:/Users/lc/Desktop/JHU_Course/CIS2/CODE/data/stereo_calib_images10/left/left_0.png';
rightImageFile = 'C:/Users/lc/Desktop/JHU_Course/CIS2/CODE/data/stereo_calib_images10/right/right_0.png';
I1 = imread(leftImageFile);
I2 = imread(rightImageFile);

figure;
subplot(1,2,1); imshow(I1); title('Original Left Image');
subplot(1,2,2); imshow(I2); title('Original Right Image');

%% 3. Rectify images using calibration
[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);

figure;
subplot(1,2,1); imshow(J1); title('Rectified Left Image');
subplot(1,2,2); imshow(J2); title('Rectified Right Image');

%% 4. Compute disparity map
% Grayscale conversion for matching
grayJ1 = rgb2gray(J1);
grayJ2 = rgb2gray(J2);

disparityRange = [0 32];  % Should be multiple of 16

disparityMap = disparitySGM(grayJ1, grayJ2, ...
    'DisparityRange', disparityRange, ...
    'UniquenessThreshold', 15);

figure;
imshow(disparityMap, disparityRange);
colormap jet; colorbar;
title('Disparity Map');

% Process NaNs in disparity map
mask = isnan(disparityMap);
disparityMapTemp = disparityMap;
disparityMapTemp(mask) = 0;
disparityMapFilled = regionfill(disparityMapTemp, mask);
disparityMap = double(disparityMapFilled);

figure;
imshow(disparityMap, disparityRange);
colormap jet; colorbar;
title('Filled Disparity Map');

filteredDisparity = wls_filter(disparityMap, 35, 0.01);
figure;
imshow(filteredDisparity, disparityRange);
colormap jet; colorbar;
title('WLS Filtered Disparity Map');

%% 5. Reproject to 3D point cloud
points3D = reconstructScene(filteredDisparity, stereoParams);
points3D = points3D ./ 1000;

ptCloud = pointCloud(points3D, 'Color', J1);
denoisedPtCloud = pcdenoise(ptCloud);
pcshow(denoisedPtCloud);

%% 6. Detect checkerboard and measure square size
[imagePointsSingle, bs] = detectCheckerboardPoints(J1);
bs = bs - [1,1];

if isempty(imagePointsSingle)
    warning('Checkerboard not detected!');
else
    figure; imshow(J1); hold on;
    plot(imagePointsSingle(:,1), imagePointsSingle(:,2), 'ro');
    title('Detected Checkerboard Corners'); hold off;

    distances = [];
    for row = 1:bs(1)
        for col = 1:bs(2)-1
            idx1 = (row-1)*bs(1) + col;
            idx2 = (row-1)*bs(1) + col + 1;
            pt1 = imagePointsSingle(idx1,:);
            pt2 = imagePointsSingle(idx2,:);

            x1 = round(pt1(1)); y1 = round(pt1(2));
            x2 = round(pt2(1)); y2 = round(pt2(2));

            if x1>0 && x1<=size(points3D,2) && y1>0 && y1<=size(points3D,1) && ...
               x2>0 && x2<=size(points3D,2) && y2>0 && y2<=size(points3D,1)
                X1 = points3D(y1,x1,1); Y1 = points3D(y1,x1,2); Z1 = points3D(y1,x1,3);
                X2 = points3D(y2,x2,1); Y2 = points3D(y2,x2,2); Z2 = points3D(y2,x2,3);
                if all(isfinite([X1,Y1,Z1,X2,Y2,Z2]))
                    d = norm([X2-X1, Y2-Y1, Z2-Z1]);
                    distances(end+1) = d;
                end
            end
        end
    end

    if isempty(distances)
        warning('No valid square distances measured.');
    else
        avgDist = mean(distances);
        avgDist_mm = avgDist * 1000;
        fprintf('Average checkerboard square length: %.2f mm\n', avgDist_mm);
    end
end

%% 7. Measure depth at image center
centerX = round(size(points3D,2)/2);
centerY = round(size(points3D,1)/2);

Zc = points3D(centerY, centerX, 3);
if isfinite(Zc)
    Zc_mm = Zc * 1000;
    fprintf('Center depth: %.2f mm\n', Zc_mm);
else
    fprintf('Invalid depth at center (NaN or Inf).\n');
end
