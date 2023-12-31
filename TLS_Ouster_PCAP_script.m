%%   Dense  point  cloud  acquisition  with  a  Ouster OS-32
%    Original Author : Jason Bula
%    Modified to work with Ouster Lidar and as a stand alone App by Donny Mott


         % Get user home directory in Unix-based systems (including Linux and Mac)
         %[status, user_home] = system('echo $HOME');

         % Get user home directory in Windows
         [status, user_home] = system('echo %USERPROFILE%');

         % Trim trailing newline character from system command output
         user_home = strtrim(user_home);

         % Set the script path
         my_settings = fullfile(user_home, 'TLS_Ouster', 'application', 'settings.mat');

         load(my_settings);

pcapFileName = pcapFileName;
calibFileName = calibFileName;

disp(pcapFileName)
disp(calibFileName)

file = pcapFileName;
[filepath,~,~] = fileparts(file);
input = filepath;
disp(input)

% Export folder
resultsDir = fullfile(input, 'results');
disp(resultsDir)
try
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end
catch ME
    disp(['Error creating directory: ', ME.message])
end

output = resultsDir;
file = pcapFileName;
[~,output_file_name,ext] = fileparts(file);
disp(output_file_name)


%% Initialization of parameters 
times = times; % Scan duration in seconds 
times_calculated = times * 10;  % times_calculated is now in tenths of a second

angle = angle; % Rotation in degrees around motor axis. Ouster TLS X+ is up, Velodyne is Y+ up

if isempty(angle) || ~isnumeric(angle) || angle < 45 || angle > 360
    disp('Input for angle is invalid or outside the valid range (45-360). Setting angle to 360.');
    angle = 360;
end

first = first; % Time at the first frame (delay)

filesToMerge = {}; %clean array

%% Export parameters
% Set to 0 to keep all the points
% Set to 1 to keep the x-positives values (needed for Ouster)
% Set to 2 to keep the y-negatives values (needed for Velodyne)
pos = pos; % set to 1 normally / Set to 0 for calibration of a2

pos2 = pos2; % set to 1 normally / Set to 0 to keep save First and last band/ Set to 1 to keep all bands/ Set to 2 to keep middle band

gridStep = gridStep; %Set the grid resolution in [m] / Set to 0 to keep all the values (subsampling)

%% Calibration parameters
% Correction angle alpha_1 and alpha_2 [degrees] Those angle are determined automatically after calibration or be chosen
alpha_1 = alpha_1; %incorrect value cause blur
alpha_2 = alpha_2; %Incorrect value causes domeing

% Arm length [m] distance rotation center of Lidar off from Motor axis
R =     R;

theta3 = 0; % adjust for irregularities or discrepancies in the speed of the motor



%% Point cloud correction to be applied during rotation
% 
%  In this part of the code, the scan will be extracted frame by frame in order to to apply the necessary correction to realign the point cloud.
%  First of all, only images containing positive X will be kept, then each image will be processed separately.
%  Two transformation matrices will be applied to each image. One containing rotation according to the LiDAR angle (varies over time) and
%  the other, an applied translation corresponding to the distance between 
%  the arm and the optical center of the LiDAR. R = R.Finally, each image is saved separately in a Cell.

% Initialisation
ousterReader = ousterFileReader(pcapFileName,calibFileName);

%count frames of scan
totalScanFrames = ousterReader.NumberOfFrames;
%disp(totalScanFrames)
fprintf('Total Number of Scan Frames (totalScanFrames): %d\n', totalScanFrames);

usableFrames = totalScanFrames - first;  % frames-start delay
%disp(usableFrames)
fprintf('Total Number of Scan Frames minus Delay (usableFrames): %d\n', usableFrames);

% Convert usableFrames to user scale and round down to nearest integer
usableTimes = floor(usableFrames / 10);  % account for user's times being 1/10th of the actual frame number

%disp(usableTimes)
fprintf('Total Number of Scan Frames minus Delay / 10 (usableTimes): %d\n', usableTimes);

% Check if 'times' exceeds the usable times
if times > usableTimes
    warning('Entered time value is too large. It has been replaced with the maximum allowed Time of %d.', usableTimes);
    times_calculated = usableFrames;  % replace times with maximum allowed value
    times = usableTimes;
end

%disp(times)
fprintf('Input Value in Time (times): %d\n', times);

%disp(times_calculated)
fprintf('times calculated: %d\n', times_calculated);

settings_file = fullfile(user_home, 'TLS_Ouster', 'application', 'settings.mat'); %Windows
save(settings_file, 'times', 'angle', 'first', 'gridStep', 'alpha_1', 'alpha_2', 'R', 'pos', 'pos2', 'pcapFileName', 'calibFileName', 'usableTimes', 'usableFrames','totalScanFrames');


last = first + times_calculated; % Time at the last frame
angle_deg=(0:angle/times_calculated:angle); % Angle after each frame
angle = deg2rad(angle_deg); % Angle in radian
s = 0;


% Initialize Cloud
%Cloud = cell(1, length(angle) - 1);

for i = 2 : length(angle) % Runs as many times as there are frames
     NF = first + s;
     ptCloudIn = readFrame(ousterReader,NF); % Selecting frames separately

% Apply export parameters 
% pos 1 filters (velodyne) the point cloud data to only include points that are
% located in the positive X half-space (X > 0). Any point cloud data with
% X-coordinate values less than or equal to zero are removed.

% pos 1 filters (ouster) filters the point cloud data to only include
% points that are located in the positive Y half-space (Y > 0). Any point
% cloud data with Y-coordinate values less than or equal to zero are
% removed.

 if pos == 1   
     
    % ptCloudIn3 =  ptCloudIn.Location(:,:,1); % extract the x-coordinate values from the point cloud (velodyne)
      ptCloudIn3 =  ptCloudIn.Location(:,:,2); % extract the y-coordinate values from the point cloud (ouster)
      ptCloudIn3(isnan(ptCloudIn3))=0; % replacing all the NaN values in the x-coordinates with zeros
     
      ptCloudIntensity = ptCloudIn.Intensity(:,:); % intensity values of the point cloud are being extracted
      ptCloudIntensity(isnan(ptCloudIntensity))=0; % any NaN values are being replaced with zeros
     
     ptCloudIn3(ptCloudIn3<0)=0; % intensity values are being multiplied by the binary mask. 
     ptCloudIn3(ptCloudIn3>0)=1; % zeroes out the intensity values for all points with an x-coordinate less than or equal to zero
     
     ptCloudIntensity2 = single(ptCloudIntensity(:,:)).*ptCloudIn3; % intensity values are being multiplied by the binary mask
     ptCloudIn = pointCloud(ptCloudIn.Location(:,:,:).*ptCloudIn3,'Intensity',ptCloudIntensity2);
 	
	% the pos variable determines which half-space of the relevant axis
    % (Velodyne) X-axis or (Ouster) Y-axis is preserved in the output point cloud. This
	% filtering process is accomplished by creating a binary mask based on the
	% sign of the coordinate values and applying this mask to the point cloud
	% coordinates and intensity values. The output point cloud only includes
	% points from the specified half-space and the rest of the points are
	% filtered out.
end

% pos 2 (velodyne) filters the point cloud data to only include points that are
% located in the negative X half-space (X < 0). Any point cloud data with
% X-coordinate values greater than or equal to zero are removed.

% pos 2 (ouster) filters the point cloud data to only include points that
% are located in the negative Y half-space (Y < 0). Any point cloud data
% with Y-coordinate values greater than or equal to zero are removed.

if pos == 2   
     
%    ptCloudIn3 =  ptCloudIn.Location(:,:,1); %(velodyne)
     ptCloudIn3 =  ptCloudIn.Location(:,:,2); % extract the y-coordinate values from the point cloud (ouster)
     ptCloudIn3(isnan(ptCloudIn3))=0; % replacing all the NaN values in the y-coordinates with zeros
     
     ptCloudIntensity = ptCloudIn.Intensity(:,:);
     ptCloudIntensity(isnan(ptCloudIntensity))=0;
     
     ptCloudIn3(ptCloudIn3>0)=0;
     ptCloudIn3(ptCloudIn3<0)=1;
     
     ptCloudIntensity2 = single(ptCloudIntensity(:,:)).*ptCloudIn3;
     ptCloudIn = pointCloud(ptCloudIn.Location(:,:,:).*ptCloudIn3,'Intensity',ptCloudIntensity2);
end

  
clear ptCloudIn2 ptCloudIn3 

% Transformation of each image
% Définition des matrices de transformation

% Alignment correction as a function of motor speed (y-axis) (velodyne)
%VM = [cos(angle(i)) 0 sin(angle(i)) 0; 0 1 0 0; -sin( angle(i)) 0 ...
%      cos(angle(i)) 0; 0 0 0 1];

% Alignment correction as a function of motor speed (x-axis) (ouster)
VM = [1 0 0 0; 0 cos(angle(i)) sin(angle(i)) 0 ; ...
      0 -sin(angle(i)) cos(angle(i)) 0 ; 0 0 0 1];

% Alpha_1 correction (y-axis)
A1 = [1 0 0 0; 0 cosd(alpha_1) sind(alpha_1) 0 ; ...
      0 -sind(alpha_1) cosd(alpha_1) 0 ; 0 0 0 1];
  
% Alpha_2 correction (z-axis)
A2 =  [cosd(alpha_2) sind(alpha_2) 0 0; ...
     -sind(alpha_2) cosd(alpha_2) 0 0; 0 0 1 0; 0 0 0 1];
   
% R correction to translate along the z-axis (velodyne and ouster)
T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 R 1];

% Speed adjustement (velodyne)
%T4 = [cosd(theta3) sind(theta3) 0 0; ...
%     -sind(theta3) cosd(theta3) 0 0; 0 0 1 0; 0 0 0 1];

% Speed adjustment (x-axis) (ouster)
%T4 = [1 0 0 0; sind(theta3) cosd(theta3) 0 0; 0 0 1 0; 0 0 0 1];
T4 = [cosd(theta3) 0 -sind(theta3) 0; 0 1 0 0; sind(theta3) 0 cosd(theta3) 0; 0 0 0 1];

 
% Apply transformation
tform_T = affine3d(T);
ptCloudIn = pctransform(ptCloudIn,tform_T);

tform_A1 = affine3d(A1);
ptCloudIn = pctransform(ptCloudIn,tform_A1);

tform_A2 = affine3d(A2);
ptCloudIn = pctransform(ptCloudIn,tform_A2);

tform_T = affine3d(T4);
curr_img = pctransform(ptCloudIn,tform_T); % configuration 2

tform_R = affine3d(VM);
Cloud{i-1} = pctransform(curr_img,tform_R); % Save in a cell

s = s + 1; % count update

end


% Initialisation of export parameters
if pos2 == 1
    bandNumber = 32;
else
    bandNumber = 2;
end

if pos2 == 2
    bandNumber = 1;
end

%% Point cloud merging 

for bande_sep = 1 : bandNumber
    
if pos2 == 1 
    iiii=bande_sep;
end

if pos2 == 0    
    Bande_calibration = [1 32];
    iiii = Bande_calibration(bande_sep);
end

if pos2 == 2    
    Bande_calibration = 16;
    iiii = Bande_calibration(bande_sep);
end

% Variable initialization
x = [];
y = [];
z = [];
int = [];

X_ref = [];
Y_ref = [];
Z_ref = [];
int_ref = [];

X_ref_final = [];
Y_ref_final = [];
Z_ref_final = [];
int_ref_final = [];

% Acceleration of the process by combining several loops
for iii = 1 : 10 
    for ii = round((times_calculated/10*iii)-((times_calculated/10)-1)) : round(iii*times_calculated/10)    
   

     for i = iiii:iiii % save the band separately

% Deleting old values
        x1 = [];
        y1 = [];
        z1 = [];
        int1 = [];
% Selection of points in the correct matrix locations
% Point clouds are recorded as follows: 32*1800*3
% 32 corresponds to the band, 1800 corresponds to the number of points recorded
% per band, 3 corresponds to the x, y and z values. 

%disp(['Size of Cloud: ', mat2str(size(Cloud))]);
%disp(['ii: ', num2str(ii)]);
%disp(['Size of Cloud{1,ii}.Location: ', mat2str(size(Cloud{1,ii}.Location))]);



x1(i,:) = Cloud{1,ii}.Location(i,:,1);
y1(i,:) = Cloud{1,ii}.Location(i,:,2);
z1(i,:) = Cloud{1,ii}.Location(i,:,3);
int1(i,:) = Cloud{1,ii}.Intensity(i,:);


x = [x x1(i,:)];
y = [y y1(i,:)];
z = [z z1(i,:)];
int = [int int1(i,:)];


    end 
    X_ref = [X_ref x];
    Y_ref = [Y_ref y];
    Z_ref = [Z_ref z];
    int_ref = [int_ref int];
    
    x = 0;
    y = 0;
    z = 0;
    int = 0;
    
    end
    X_ref_final = [X_ref_final X_ref];
    Y_ref_final = [Y_ref_final Y_ref];
    Z_ref_final = [Z_ref_final Z_ref];
    int_ref_final = [int_ref_final int_ref];
    
% disp(iii) % Compteur de progression de l'extraction 
    X_ref = 0;
    Y_ref = 0;
    Z_ref = 0;
    int_ref = 0;
end
        

%% Reconstruction of the point cloud
ref = [X_ref_final; Y_ref_final; Z_ref_final]'; 
   
PC_corr1 = pointCloud(ref,'Intensity',int_ref_final');

if gridStep == 0
    PC_downsampled_1 = PC_corr1;
else
    PC_downsampled_1 = pcdownsample(PC_corr1,'gridAverage',gridStep);
end

% Remove invalid values
[PC_Final1,indices]= removeInvalidPoints(PC_downsampled_1);

% Create the rotation matrix (velodyne)
%RotX = [1 0 0 0; 0 cosd(-90) -sind(-90) 0; 0 sind(-90) cosd(-90) 0; 0 0 0 1];

% Apply the rotation to point cloud to align Z axis upwards (velodyne)
%PC_Final1 = pctransform(PC_Final1, affine3d(RotX));

% Create the rotation matrix (ouster)
RotY = [cosd(-90) 0 sind(-90) 0; 0 1 0 0; -sind(-90) 0 cosd(-90) 0; 0 0 0 1];

% Apply the rotation to point cloud to align Z axis upwards (ouster)
PC_Final1 = pctransform(PC_Final1, affine3d(RotY));

%% Point cloud export Seperate Files plus merged file

% Defining output document names
try
filename = sprintf('%s_%d.ply', output_file_name, iiii);

% Change directory to output
cd(output)

%Write file
pcloud{1,ii} = PC_Final1;
pcwrite(PC_Final1,filename,'PLYFormat','binary');

% Store filenames for later merging
filesToMerge{bande_sep} = fullfile(output, filename);
disp(['File stored for merging: ', filesToMerge{bande_sep}]);

% Return to input directory
cd(input)
ref = []; % suppression of the loaded point cloud
f = msgbox((["Processed File:";output_file_name,'_', num2str(iiii) ' of 32\n']),"Status");
pause(1)
if isvalid(f); delete(f); end
catch ME
    disp(['Error writing file: ', ME.message])
end
end
% Define merge tolerance
%mergeTolerance = gridStep unless gridStep is 0 then subsample at 0.005
%meters. (for pcmerge gridstep has to be a positive value)
if gridStep ~= 0
    mergeTolerance = gridStep; 
else
    mergeTolerance = 0.005;  
end
 
% Check if there are multiple files to merge Add
if length(filesToMerge) > 1 

% Read the first file and use it to initialize the merged point cloud
tempPC = pcread(filesToMerge{1});
mergedPointCloud = tempPC;

% Read each subsequent file back in and merge
for i = 2:length(filesToMerge)
    tempPC = pcread(filesToMerge{i});
    mergedPointCloud = pcmerge(mergedPointCloud, tempPC, mergeTolerance);
end

% Define the output filename
merged_filename = [output_file_name,'_merged.ply'];

% Save the merged point cloud
pcwrite(mergedPointCloud, fullfile(output, merged_filename), 'PLYFormat', 'binary');
disp(['Merged File Saved: ', merged_filename]);

% Display the merged point cloud

% Set the maximum limit of points for visualization
    maxNumPoints = 5e5; % 100,000 points would be 1e5, 500,000 would be 5e5 and 1,000,000 is to many points.
    if mergedPointCloud.Count > maxNumPoints
        % Downsample the point cloud if it contains more than the maximum limit
        mergedPointCloud = pcdownsample(mergedPointCloud, 'random', maxNumPoints / mergedPointCloud.Count);
    end

    % Normalize the Z values to the range [0, 1]
    z = mergedPointCloud.Location(:, 3); % Z values
    z = (z - min(z)) / (max(z) - min(z)); % normalization to [0, 1]

    % Apply the colormap
    cmap = jet(256); % colormap
    c = round(z * (size(cmap, 1) - 1)) + 1; % Match color indices to z values
    color = uint8(cmap(c, :) * 255); % Convert to 8-bit RGB color

    % Create a new point cloud with color
    mergedPointCloudColor = pointCloud(mergedPointCloud.Location, 'Color', color);

    % Apply the colormap
    pcshow(mergedPointCloudColor);
    title('Merged Point Cloud');
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
else
    % Specify action for the case of one file
    disp('Only one file, skipping merging process.')

    % Read the single point cloud file
    singlePointCloud = pcread(filesToMerge{1});

    % Normalize the Z values to the range [0, 1]
    z = singlePointCloud.Location(:, 3); % Z values
    z = (z - min(z)) / (max(z) - min(z)); % normalization to [0, 1]

    % Apply the colormap
    cmap = jet(256); % colormap
    c = round(z * (size(cmap, 1) - 1)) + 1; % Match color indices to z values
    color = uint8(cmap(c, :) * 255); % Convert to 8-bit RGB color

    % Create a new point cloud with color
    singlePointCloudColor = pointCloud(singlePointCloud.Location, 'Color', color);

    % Display the single point cloud
    pcshow(singlePointCloudColor);
    title('Single Point Cloud');
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
end
% Return to input directory
cd(input)


