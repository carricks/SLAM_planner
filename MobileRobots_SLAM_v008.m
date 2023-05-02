%% Testing for SLAM proyect 

clc;
clear all;

% ---- ---- ---- ---- --- ---- ---- ---- ----
% ---- ---- ---- ---- MAP ---- ---- ---- ----
% ---- ---- ---- ---- --- ---- ---- ---- ----

% Read the original map from CSV file.
mapMatrix = readmatrix('./maps/map2.csv');

% Define starting position
start = [0, 0, pi];

% The grid size is (1m*1m)
resolution = 1; 

% Each map has different grid dimensions but they are all 2D.
% Initialize a map for scanning results storage.
worldMap = binaryOccupancyMap(mapMatrix, resolution);
worldMap.GridLocationInWorld = [ -8 -2];
% Display scaneable map and the theorical resulting map
OriginalMap = figure('Name','Original Map');
figure(1);
show(worldMap);


% ---- ---- ---- ---- ----- ---- ---- ---- ----
% ---- ---- ---- ---- Robot ---- ---- ---- ----
% ---- ---- ---- ---- ----- ---- ---- ---- ----
% Robot real state in environment

rPos = start;
roboR = 0.15 ; % 15cm of radious

% ---- ---- ---- ---- ----- ---- ---- ---- ----
% ---- ---- ---- ---- LiDAR ---- ---- ---- ----
% ---- ---- ---- ---- ----- ---- ---- ---- ----
% LiDAR -- Range Sensor
lidar = rangeSensor;
lidar.Range = [0 5]; % 5m of max range
lidar.RangeNoise = 0.05;

% Define SLAM for lidar
maxLidarRange = 5;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

% These parameters are set "empirically" according to the MATLAB Navigation
% Toolbox tutorials
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

% ---- ---- ---- ---- --- ---- ---- ---- ----
% ---- ---- ---- ---- IMU ---- ---- ---- ----
% ---- ---- ---- ---- --- ---- ---- ---- ----
% IMU -- Get to thine position

% Define IMU to have an aacelerometer and a gyroscope 
imu = imuSensor('accel-gyro');
imu.Gyroscope.NoiseDensity = [0.05 0.05 0.05];
imu.Accelerometer.NoiseDensity = [0.05 0.05 0.05];
imu.SampleRate = 20;

% ---- ---- ---- ---- --------- ---- ---- ---- ----
% ---- ---- ---- ---- DiffDrive ---- ---- ---- ----
% ---- ---- ---- ---- --------- ---- ---- ---- ----

%Declare the main motion drive of the robot
diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");
% Define a Pure Pursuit controller
controller = controllerPurePursuit('DesiredLinearVelocity',1,'MaxAngularVelocity',1);
%Set waitpoints
waypoints = [ -4 0; -4 4];
% Define for Pure Pursuit controller
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.3;

% ---- ---- ---- ---- ---------- ---- ---- ---- ----
% ---- ---- ---- ---- Simulation ---- ---- ---- ----
% ---- ---- ---- ---- ---------- ---- ---- ---- ----

% Figure object for robot visualization
ax1 = OriginalMap.CurrentAxes;

% Define sampleTime and time dimension for simulation
sampleTime = 0.05;          % Sample time [s]
time = 0:sampleTime:100;       % Time array
rate = rateControl(1/sampleTime);

% Generate poses matrix for a position in each iteration loop
% ground truths
poses = zeros(3,numel(time));    % Pose matrix
poses(:,1) = rPos';
velPast = [0;0;0];

% Variables for storing Acceleration, Velocity, Position, Angular Velocity
% and Angle for the robot.
acc_odom = zeros(3,numel(time));
vel_odom = zeros(3,numel(time));
dist_odom = zeros(3,numel(time));
wTheta_odom = zeros(1,numel(time));
theta_odom = zeros(1,numel(time));
odomPose = [0;0;0];
odomPoses = [];

% Simulation loop
goal = waypoints(end,:)';
scans = {}; % erase???
mapPoints = []; 

for t = 1:numel(time)
    
    position = poses(:,t)';
    current_pos = position(1:2);
    
    % End if pathfollowing is vehicle has reached goal position within tolerance of 0.2m
        dist = norm(goal'-current_pos);
        if (dist < 0.5)
            disp("Goal position reached")
            break;
        end
        
    % ---- ---- ----
    % ---- SLAM ----
    % ---- ---- ----
    % Scan current position with LiDAR
    [ranges, angles] = lidar(position,worldMap);
    scan = lidarScan(ranges,angles);
    scans{t} = scan;
        
    % SLAM with scans
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{t});
    if isScanAccepted
        fprintf('Added scan %d \n', t);
        figure(2);
        hold on
        show(slamAlg);
    end
    
    % ---- ------- ----
    % ---- Control ----
    % ---- ------- ----

    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(poses(:,t));
    
    % Perform forward discrete integration step
    vel = derivative(diffDrive, poses(:,t), [vRef wRef]);
    poses(:,t+1) = poses(:,t) + vel*sampleTime; 
    
    % ---- ---- ----
    % ---- Odom ----
    % ---- ---- ----
    
    % Derivate velocity to get acceleration
    dv = diff([velPast vel],1,2);
    a = dv/sampleTime;
    velPast = vel;
    
    % Read IMU
    [accRead, gyroRead] = imu(a',[0 0 wRef]);
    acc_odom(:,t) = accRead';
    wTheta_odom(t)= gyroRead(3);
    
    % Get odomPose
    vel_odom = cumtrapz(time,acc_odom,2); % Velocity in XY
    dist_odom =  cumtrapz(time,vel_odom,2); % Position in XY
    theta_odom = cumtrapz(time,wTheta_odom); % Angle theta of robot
    
    % Define odomPose with [PositionX PositionY angVel]
    odomPose = dist_odom(:,t);
    odomPose(3) = theta_odom(t);
    odomPoses = [odomPoses; position];
    
    % ---- ------- ----
    % ---- Voronoi ----
    % ---- ------- ----
    % Build the map with the scans and the poses acquired during SLAM
    pose = slamAlg.PoseGraph.nodes;
    pose = pose(t,:);
    % Correct the information in the scan to the real location in a map
    cScan = scanCorrection(scan,pose);
    cScan(any(isnan(cScan), 2),:)= [];
    % Add point to a the map for obstacle locations
    mapPoints = [mapPoints;cScan];
    % Calculate Voronoi points for current corrected scan in order to
    % choose randomly a point of destination
    vorPts = calcVor(cScan, pose);
    
    %Plot map and calculated Voronoi
    figure(3);
    plot(mapPoints(:,1),mapPoints(:,2),'.')
    hold on
    plot(vorPts(:,1),vorPts(:,2),'o')
    hold off
    
    % Update visualization
    plotTrvec = [poses(1:2, t+1); 0];
    plotRot = axang2quat([0 0 1 poses(3, t+1)]);

    % Delete image of the last robot to prevent displaying multiple robots
    if t > 1
    items = get(ax1, 'Children');
    delete(items(1)); 
    end  
    
    % Plot robot onto known map
    plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 0.25, 'Parent', ax1);
    
    waitfor(rate);
end
    
%%

function vorPts = calcVor(cScan, pose)

% Calculate delaunay triangulation for Voronoi Diagram
delPts = delaunayTriangulation(cScan);
% Get the voronoi diagram points for planning
[vorPts, ~] = voronoiDiagram(delPts);

% If there is an infinite point calculated by the previous function, get
% rid of it
if(vorPts(1,2) == Inf)
    vorPts = vorPts(2:end,:);
end

% Delete all calculated points farther away than 4m
for i=1:length(vorPts)
    if( norm( vorPts(i,:) - pose(1:2) ) > 4 )
        vorPts(i,:) = [0 0];
    end
end
vorPts( ~any(vorPts,2), :) = [];

% Delete any remaning point that is closer than 1m
for i=1:length(vorPts)

    for j=1:length(cScan)
        dist = norm(vorPts(i,:)-cScan(j,:));
      
        if( dist < 1 )
            vorPts(i,:) = [0 0];
        end
    end

end

% Delete the zeros from Voronoi Points
vorPts( ~any(vorPts,2), :) = [];

end

function  newscan = scanCorrection(scan, pose)

% Corrected scan points for voronoiPts
newscan = [];

% Get cartesian pts from scans
scan = scan.Cartesian;
scan = [ scan ones(length(scan),1)];

% Translation matrix
T = [1       0       pose(1); ... 
     0       1       pose(2); ... 
     0       0       1;];
% Rotation matrix
R = [cos(pose(3))   -sin(pose(3))     0; ... 
     sin(pose(3))   cos(pose(3))      0; ... 
     0              0                 1;];
 
% Point transformation loop
    for i=1:length(scan)
    
        newscan = [newscan ; (T*(R*scan(i,:)'))' ];
    
    end

% Choose only   
newscan = newscan(:,1:2);

end
 

