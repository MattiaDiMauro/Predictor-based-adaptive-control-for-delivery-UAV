
function [y, yd, ydd, yddd, ydddd, pp, tPoints, tSamples] = PlanningPathFunction()

addpath map_tools

%%
 %Configure the random number generator for repeatable result.
 rng("default")
 % Map geometry and grid resolution
 r = 0.215; % drone radius: parameter to be used in inflate map routine
 % Mapsize
 mapsize.x = 80; %[m]
 length_ground = mapsize.x;
 mapsize.y = 50; %[m]
 width_ground = mapsize.y;
 % Map resolution
 res = 1; %[m]
 grid_size = res;
 height_ground = res;

 % Create map
 map3D = occupancyMap3D(1/res);  %1/res cells per meter
 map3D.FreeThreshold = map3D.OccupiedThreshold; % set to 0.65; unknown spaces (occupancy 0.5) considered as free
 % Define Ground Plane 
for x = -length_ground/2+res/2:res:length_ground/2-res/2
    for y = -width_ground/2+res/2:res:width_ground/2-res/2
        for z = -height_ground
           xyz = [x y z];
           setOccupancy(map3D, xyz, 1) 
       end
    end
end

show(map3D)

%%
% Insert boxes 1
length_box = 20; % use multiple of the cell dimensions
width_box = 20;
height_box = 10;
% Box 1 
pos_x1 = -30; % use multiple of the cell dimensions
pos_y1 = -15;
pos_z1 = 5;
create_box(res, pos_x1, pos_y1, pos_z1,length_box, width_box, height_box, map3D)
show(map3D)

% Insert boxes 2
length_box = 10; % use multiple of the cell dimensions
width_box = 30;
height_box = 10;
% Box 1 
pos_x1 = -5; % use multiple of the cell dimensions
pos_y1 = 5;
pos_z1 = 5;
create_box(res, pos_x1, pos_y1, pos_z1,length_box, width_box, height_box, map3D)
show(map3D)


% Insert boxes 3
length_box = 10; % use multiple of the cell dimensions
width_box = 10;
height_box = 10;
% Box 1 
pos_x1 = 10; % use multiple of the cell dimensions
pos_y1 = -20;
pos_z1 = 5;
create_box(res, pos_x1, pos_y1, pos_z1,length_box, width_box, height_box, map3D)
show(map3D)


% Insert boxes 2
length_box = 10; % use multiple of the cell dimensions
width_box = 30;
height_box = 10;
% Box 1 
pos_x1 = 25; % use multiple of the cell dimensions
pos_y1 = 5;
pos_z1 = 5;
create_box(res, pos_x1, pos_y1, pos_z1,length_box, width_box, height_box, map3D)
show(map3D)


% Insert boxes 5
length_box = 10; % use multiple of the cell dimensions
width_box = 10;
height_box = 10;
% Box 1 
pos_x1 = 35; % use multiple of the cell dimensions
pos_y1 = -20;
pos_z1 = 5;
create_box(res, pos_x1, pos_y1, pos_z1,length_box, width_box, height_box, map3D)
show(map3D)

%% inflate the map with the drone radius
 inflate(map3D,r)
 show(map3D)

 %% Starting position
 startState = [-35, -2.5, 0];
 % Target position
 goalState = [32.5, 15, 0];
 % Plot map, start pose, and goal pose
 show(map3D)
 hold on
 scatter3(startState(1),startState(2),startState(3),30,".r")
 scatter3(goalState(1),goalState(2),goalState(3),30,".g")

%% Define state space
xmax = mapsize.x/2;
ymax = mapsize.y/2;
zmax = 11;
ss = E3space([-xmax xmax;-ymax ymax;0 zmax]);
% Define validator occupancy object
sv = validatorOccupancyMap3D(ss);
sv.ValidationDistance = res; 
sv.Map = map3D;


%% Plan the path
planner = plannerRRTStar(ss,sv); 
%tunable parameters
planner.ContinueAfterGoalReached = true;
planner.MaxConnectionDistance = 3.5; 
planner.GoalBias = 0.25; %between 0 and 1
planner.MaxIterations = 10000;
planner.MaxNumTreeNodes = 10000;
%planner.BallRadiusConstant = 100 ; % (default) read the documentation
[pthObj,solnInfo] = plan(planner, startState, goalState);
if (~solnInfo.IsPathFound)
    disp("No Path Found by the RRT, terminating example")
    return
end

% Plot the waypoints
plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3),"-g")
view([-31 63])
legend("","Start Position","Goal Position","Planned Path")


%% Generate trajectory
waypoints = pthObj.States(:,1:3);
nWayPoints = pthObj.NumStates;
% Calculate the distance between waypoints
distance = zeros(1,nWayPoints);
for i = 2:nWayPoints
distance(i) = norm(waypoints(i,1:3) - waypoints(i-1,1:3));
end
% Assume a UAV speed of 1 m/s and calculate time taken to reach each waypoint
UAVspeed = 1.13;   
timepoints = cumsum(distance/UAVspeed);
nSamples = 200;
% Compute states along the trajectory (minsnappolytraj )
    [y,yd,ydd,yddd,ydddd,pp,tPoints,tSamples] = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=100);
initialStates = y';
% Plot the minimum snap trajectory
plot3(initialStates(:,1),initialStates(:,2),initialStates(:,3),"-y")
legend("","Start Position","Goal Position","Planned Path","Initial Trajectory")


% check that the trajectory does not hit the obstacles
states = y';
valid = all(isStateValid(sv,states)); 
if (~valid)
    disp("Trajectory not feasible, adding intermediate waypoints")
end

% %% Generate trajectory: Alternative solution using time weight
% % Compute states along the trajectory (minsnappolytraj with time weight)
% [y,yd,ydd,yddd,ydddd,pp,tPoints,tSamples] = 
% minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=1);


%% Generate feasible minimum snap trajectory by adding intermediate waypoints
while(~valid)
    % Check the validity of the states
    validity = isStateValid(sv,states);
    % Map the states to the corresponding waypoint segments
    segmentIndices = MapStatesToPathSegments(waypoints,states);
    % Get the segments for the invalid states
    % Use unique, because multiple states in the same segment might be invalid
    invalidSegments = unique(segmentIndices(~validity));
    % Add intermediate waypoints on the invalid segments
    for i = 1:numel(invalidSegments)
        segment = invalidSegments(i);
        % Take the midpoint of the position to get the intermediate position
        midpoint = (waypoints(segment,:) + waypoints(segment+1,:))/2;
    end
    nWayPoints = size(waypoints,1);
    distance = zeros(1,nWayPoints);
    for i = 2:nWayPoints
        distance(i) = norm(waypoints(i,1:3) - waypoints(i-1,1:3));
    end
    % Calculate the time taken to reach each waypoint
    timepoints = cumsum(distance/UAVspeed);
    [y,yd,ydd,yddd,ydddd,pp,tPoints,tSamples] = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=1);
    states = y';
    % Check if the new trajectory is valid
    valid = all(isStateValid(sv,states));
end


save('poly_coeff','pp')
 % Plot the final valid trajectory
 plot3(states(:,1),states(:,2),states(:,3),"-c")
 view([-31 63])
 legend("","Start Position","Goal Postion","Planned Path","Initial Trajectory","Valid Trajectory")
 hold off
%% plot velocity and acceleration to check dynamic feasibility
figure
plot(tSamples,yd)
grid on
xlabel('Time [s]')
ylabel("Velocity [m/s]")
figure
plot(tSamples,ydd)
grid on
xlabel('Time [s]')
ylabel("Acceleration [m/s2]")

%% Prepare reference trajectory for Simulink
% Time vector
time = tSamples';

% Position desired [x, y, z]
pd = y';  % (nSamples x 3)

% Velocity desired [vx, vy, vz]
vd = yd';  % (nSamples x 3)

% Acceleration desired [ax, ay, az]
ad = ydd';  % (nSamples x 3)

% Calculate desired yaw angle (psid)
psid = ones(length(time), 1)*15*pi/180;

% Create structure for trajectory planner
trajectory_data.time = time;
trajectory_data.pd = pd;
trajectory_data.vd = vd;
trajectory_data.ad = ad;
trajectory_data.psid = psid;

% Save to MAT file
save('trajectorypath_data.mat', 'trajectory_data');

end