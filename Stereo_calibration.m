%% 双目相机标定与匹配（Calibration and Matching）
% 本脚本将完成：
% 1. 从指定文件夹中加载左右图像；
% 2. 利用 detectCheckerboardPoints 检测棋盘格角点；
% 3. 使用 estimateStereoParameters 完成双目标定，获得左右内参及双目外参；
% 4. 对一对图像进行校正（rectify）；
% 5. 利用 disparitySGM 计算视差图；
% 6. 依据标定结果生成 Q 矩阵，并重投影生成深度图和 3D 点云；
% 7. 显示视差图和点云。

clear; clc; close all;

%% 1. 参数设置
% 文件夹路径（请修改为你存放图片的目录）
imageFolder = 'C:\Users\lc\Desktop\JHU_Course\CIS2\stereo_calib_images';  % 示例：'C:\stereo_images'
% 棋盘格参数（内角点数以及每格尺寸，单位：米）
checkerboardSize = [10, 9];  % 内角点数
squareSize = 0.006;          % 每格边长（米）

%% 2. 加载左右图像
% 左图文件名：left_*.png, 右图文件名：right_*.png
leftFiles = dir(fullfile(imageFolder, 'left_*.png'));
rightFiles = dir(fullfile(imageFolder, 'right_*.png'));

if isempty(leftFiles) || isempty(rightFiles)
    error('未在指定目录中找到左右图像！');
end

% 生成完整的文件路径列表
numPairs = min(numel(leftFiles), numel(rightFiles));
leftImageFiles = cell(numPairs,1);
rightImageFiles = cell(numPairs,1);
for i = 1:numPairs
    leftImageFiles{i} = fullfile(imageFolder, leftFiles(i).name);
    rightImageFiles{i} = fullfile(imageFolder, rightFiles(i).name);
end

%% 3. 检测棋盘格角点
% 使用 MATLAB 内置函数 detectCheckerboardPoints
% 当传入左右图像文件列表时，该函数返回 3D 数组：
% imagePoints 为 [numPoints x 2 x numImagePairs]
[imagePoints, boardSize, pairsUsed] = detectCheckerboardPoints(leftImageFiles, rightImageFiles);
% 如果有部分图像未成功检测，则 pairsUsed 为逻辑向量
fprintf('检测到 %d 对图像中，有 %d 对成功检测到棋盘格角点。\n', numPairs, sum(pairsUsed));

% 如果有未检测成功的图像对，则剔除
leftImageFiles = leftImageFiles(pairsUsed);
rightImageFiles = rightImageFiles(pairsUsed);
imagePoints = imagePoints(:,:,pairsUsed);

% 生成棋盘格世界坐标（单位：米）
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

%% 4. 双目标定
% 直接调用 estimateStereoParameters
stereoParams = estimateStereoParameters(imagePoints(:,1,:), imagePoints(:,2,:), worldPoints);

% 显示标定结果信息
fprintf('双目标定完成，重投影均方根误差: %.4f 像素\n', stereoParams.MeanReprojectionError);
disp('左右相机的内参：');
disp(stereoParams.CameraParameters1.Intrinsics);
disp(stereoParams.CameraParameters2.Intrinsics);

%% 5. 图像校正和视差计算
% 选择一对图像进行验证（例如第一对）
I1 = imread(leftImageFiles{1});
I2 = imread(rightImageFiles{1});

% 对图像进行校正
[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);

% 显示校正后图像
figure; 
subplot(1,2,1); imshow(J1); title('校正后左图');
subplot(1,2,2); imshow(J2); title('校正后右图');

% 将彩色图转换为灰度图（匹配时建议使用灰度图）
grayJ1 = rgb2gray(J1);
grayJ2 = rgb2gray(J2);

% 计算视差图（采用半全局匹配 SGM 算法）
% DisparityRange 需要根据实际情况调整，此处设为 [0 64]（注意必须为 16 的倍数）
disparityRange = [0 64];
disparityMap = disparitySGM(grayJ1, grayJ2, 'DisparityRange', disparityRange);

% 显示视差图
figure;
imshow(disparityMap, disparityRange);
title('视差图');
colormap jet; colorbar;

%% 6. 重投影生成深度图和 3D 点云
% 生成 Q 矩阵
% 这里参考常用公式：
%   Q = [ 1    0    0   -cx;
%         0    1    0   -cy;
%         0    0    0    f;
%         0    0 -1/Tx    0];
% 其中，f 和 (cx,cy) 来自左相机内参，
% Tx 为左右相机的平移向量（取第一个分量，注意符号取反）
f = stereoParams.CameraParameters1.Intrinsics.FocalLength(1);
cx = stereoParams.CameraParameters1.Intrinsics.PrincipalPoint(1);
cy = stereoParams.CameraParameters1.Intrinsics.PrincipalPoint(2);
Tx = -stereoParams.TranslationOfCamera2(1);  % 注意：TranslationOfCamera2 为 [x y z]（单位：米）
Q = [1, 0, 0, -cx;
     0, 1, 0, -cy;
     0, 0, 0,  f;
     0, 0, -1/Tx, 0];

% 重投影生成 3D 点云（单位：米）
points3D = reconstructScene(disparityMap, Q);
% 注意：reconstructScene 返回的单位与 Q 的单位一致，此处 worldPoints 用的是米

% 将点云转换为点云对象，并去除无效点（视差为 0 或 NaN 的点）
points3D = points3D ./ 1;  % 此处无单位转换，如有需要自行调整
% 生成点云时将无穷大或 NaN 的点剔除
mask = isfinite(points3D(:,:,1)) & isfinite(points3D(:,:,2)) & isfinite(points3D(:,:,3)) & (disparityMap > disparityRange(1));
x = points3D(:,:,1);
y = points3D(:,:,2);
z = points3D(:,:,3);
ptCloudPts = [x(mask), y(mask), z(mask)];
% 获取颜色信息
colorImage = J1;  % 取左校正图作为颜色
colorImage = reshape(colorImage, [], 3);
ptCloudColors = colorImage(mask(:), :);
ptCloud = pointCloud(ptCloudPts, 'Color', ptCloudColors);

% 显示 3D 点云
figure;
pcshow(ptCloud, 'VerticalAxis','Y','VerticalAxisDir','down');
title('重构的3D点云');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

%% 7. 保存标定参数（可选）
save('stereoParams.mat', 'stereoParams');
