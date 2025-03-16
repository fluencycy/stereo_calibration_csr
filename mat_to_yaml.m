% Auto-generated by stereoCalibrator app on 15-Mar-2025
%-------------------------------------------------------


% Define images to process
imageFileNames1 = {'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_0.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_1.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_10.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_12.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_14.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_15.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_16.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_17.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_2.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_22.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_28.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_29.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_3.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_33.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_4.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_5.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_6.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_7.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_8.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\left\left_9.png',...
    };
imageFileNames2 = {'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_0.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_1.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_10.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_12.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_14.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_15.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_16.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_17.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_2.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_22.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_28.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_29.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_3.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_33.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_4.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_5.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_6.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_7.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_8.png',...
    'C:\Users\lc\Desktop\JHU_Course\CIS2\CODE\data\stereo_calib_images10\right\right_9.png',...
    };

% Detect calibration pattern in images
detector = vision.calibration.stereo.CheckerboardDetector();
minCornerMetric = 0.150000;
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames1, imageFileNames2, 'MinCornerMetric', minCornerMetric);

% Generate world coordinates for the planar pattern keypoints
squareSize = 10.000000;  % in millimeters
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);

% Read one of the images from the first stereo pair
I1 = imread(imageFileNames1{1});
[mrows, ncols, ~] = size(I1);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(stereoParams);

% Visualize pattern locations
h2=figure; showExtrinsics(stereoParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, stereoParams);

% You can use the calibration data to rectify stereo images.
I2 = imread(imageFileNames2{1});
[J1, J2, reprojectionMatrix] = rectifyStereoImages(I1, I2, stereoParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('StereoCalibrationAndSceneReconstructionExample')
% showdemo('DepthEstimationFromStereoVideoExample')


%% 5.  save to YAML 
K1 = stereoParams.CameraParameters1.Intrinsics.IntrinsicMatrix';
%  [k1, k2, p1, p2, k3]
radial1     = stereoParams.CameraParameters1.Intrinsics.RadialDistortion;
tangential1 = stereoParams.CameraParameters1.Intrinsics.TangentialDistortion;
if numel(radial1) < 3
    radial1(3) = 0;
end
distCoeffs1 = [radial1(1:2), tangential1, radial1(3)];

K2 = stereoParams.CameraParameters2.Intrinsics.IntrinsicMatrix';
% [k1, k2, p1, p2, k3]
radial2     = stereoParams.CameraParameters2.Intrinsics.RadialDistortion;
tangential2 = stereoParams.CameraParameters2.Intrinsics.TangentialDistortion;
if numel(radial2) < 3
    radial2(3) = 0;
end
distCoeffs2 = [radial2(1:2), tangential2, radial2(3)];


R_stereo = stereoParams.RotationOfCamera2;  % 3x3
T_stereo = stereoParams.TranslationOfCamera2;  % 1x3

% size
imageSize = stereoParams.CameraParameters1.ImageSize; % [height, width]
image_width  = imageSize(2);
image_height = imageSize(1);

% YAML name
yamlFilename_left = 'left.yaml';
writeCalibrationToYAML(yamlFilename_left, image_width, image_height, 'left', K1, distCoeffs1, R_stereo, T_stereo);

fprintf('Clibration saved to  %s\n', yamlFilename_left);

%  YAML name
yamlFilename_right = 'right.yaml';
writeCalibrationToYAML(yamlFilename_right, image_width, image_height, 'right', K2, distCoeffs2, R_stereo, T_stereo);

fprintf('Clibration saved to  %s\n', yamlFilename_right);

%% --- write to YAML  ---
function writeCalibrationToYAML(filename, image_width, image_height, camera_name, K, distCoeffs, R_stereo, T_stereo)
    fid = fopen(filename, 'w');
    if fid == -1
        error('can not open %s 。', filename);
    end
    
    function printMatrix(fid, fieldName, M)
        fprintf(fid, '%s:\n', fieldName);
        fprintf(fid, '  rows: %d\n', size(M,1));
        fprintf(fid, '  cols: %d\n', size(M,2));
        fprintf(fid, '  data: [');
        data = reshape(M', 1, []);
        for j = 1:length(data)
            if j < length(data)
                fprintf(fid, '%.5f, ', data(j));
            else
                fprintf(fid, '%.5f', data(j));
            end
        end
        fprintf(fid, ']\n');
    end

    fprintf(fid, 'image_width: %d\n', image_width);
    fprintf(fid, 'image_height: %d\n', image_height);
    fprintf(fid, 'camera_name: %s\n', camera_name);
    
    printMatrix(fid, 'camera_matrix', K);
    
    fprintf(fid, 'distortion_model: plumb_bob\n');
    fprintf(fid, 'distortion_coefficients:\n');
    fprintf(fid, '  rows: 1\n');
    fprintf(fid, '  cols: %d\n', numel(distCoeffs));
    fprintf(fid, '  data: [');
    for j = 1:length(distCoeffs)
        if j < length(distCoeffs)
            fprintf(fid, '%.5f, ', distCoeffs(j));
        else
            fprintf(fid, '%.5f', distCoeffs(j));
        end
    end
    fprintf(fid, ']\n');
    
    printMatrix(fid, 'R_stereo', R_stereo);
    
    fprintf(fid, 'T_stereo:\n');
    fprintf(fid, '  rows: 1\n');
    fprintf(fid, '  cols: %d\n', numel(T_stereo));
    fprintf(fid, '  data: [');
    for j = 1:length(T_stereo)
        if j < length(T_stereo)
            fprintf(fid, '%.5f, ', T_stereo(j));
        else
            fprintf(fid, '%.5f', T_stereo(j));
        end
    end
    fprintf(fid, ']\n');
    
    fclose(fid);
end