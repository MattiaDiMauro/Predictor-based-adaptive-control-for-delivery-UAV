 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A3S EXAM SCRIPT AY 2025/2026
% Author:  Davide Invernizzi (davide.invernizzi@polimi.it)
% v09/10/2025      
% This file contains data for the exam of the A3S course ay 2025/2026.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
clc;

%% Instructions
% Task 1.2
    % Set all disturbances (m_box, f_ex, pert) to 0
    % Set flag_adaptive to 0
    % Set degrad_flag to 0
    % Set trim_flag to 1
    % Set traj to 2
    % Set simulation_time to 300

% Task 1.3
    % Set all disturbances (m_box, f_ex, pert) to 0
    % Set flag_adaptive to 0
    % Set degrad_flag to 0
    % Set trim_flag to 2
    % Set traj to 4
    % Set simulation_time to 300

% Task 2.2/2.3.1 (setpoint tracking)
    % Set all disturbances (m_box, f_ex, pert) to 0
    % Set flag_adaptive to 0
    % Set degrad_flag to 0
    % Set trim_flag to 0
    % Set traj to 2
    % Set simulation_time to 300

% Task 2.3.1 (spiral tracking)
    % Set all disturbances (m_box, f_ex, pert) to 0
    % Set flag_adaptive to 0
    % Set degrad_flag to 0 
    % Set trim_flag to 0
    % Set traj to 0
    % Set simulation_time to 300

% Task 2.3.2 (spiral with degradation of effectiveness)
    % Set all disturbances (m_box, f_ex, pert) to 0
    % Set flag_adaptive to 0
    % Set degrad_flag to 1
    % Set trim_flag to 0
    % Set traj to 0
    % Set simulation_time to 300

% Task 3.1 (spiral with degradation and disturbances)
    % Set m_box to 0.4
    % Set f_ex to 1
    % Set t_pert to the desired one (if it is too late saturation can be
        % reached)
    % Remain pert to 0 (its change does not cause meaningfull changes)
    % Set flag_adaptive to 0
    % Set degrad_flag to 1
    % Set trim_flag to 0
    % Set traj to 0
    % Set simulation_time to 300
% Task 3.3 
    % Set m_box to 0 or 0.4
    % Set f_ex to 1
    % Set t_pert to the desired one (if it is too late saturation can be
        % reached)
    % Remain pert to 0 (its change does not cause meaningfull changes)
    % Set flag_adaptive to 1
    % Set degrad_flag to 1
    % Set trim_flag to 0
    % Set traj to 0
    % Set simulation_time to 300

% Task 4.1 
    % Set m_box to 0.4
    % Set f_ex to 1
    % Set t_pert to the desired one (if it is too late saturation can be
        % reached)
    % Remain pert to 0 (its change does not cause meaningfull changes)
    % Set flag_adaptive to 1
    % Set degrad_flag to 1
    % Set trim_flag to 0
    % Set traj to 5
    % Set simulation_time to 100



%% Tunable parameters - Important parameters that can be changed

simulation_time=100; 


% Disturbances
% Package
m_box=0.4; 
x_c=0.01; % Position of the package with respect to the body frame of the drone
y_c=-0.01;
z_c=-0.075;

% Forces
f_ex=1; % Gaussian noise of variance f_ex/20 + Gust of 5 seconds of intensity f_ex
t_pert=50; % Gust time
lambda_r=1; % Initial effectiveness 
pert=0*[1 1 1 0 0 0]'; % Extra drag coefficients (another perturbation)

% Control parameters 
traj=5; % 0: SPIRAL || 1: CIRCULAR || 2: SETPOINT TRACKING || 3: LINEAR || 4: HORIZONTAL v=[2 0 0]' || 5: realistic scenario
flag_adaptive=1;  % 0: without the adaptive control || 1: with the adaptive control
degrad_flag=1; % 1: 40% of linear degradation in 500 secs || Anything else: no degradation
trim_flag=0; % 0: Control input || 1: position trim || 2: velocity trim


%% Initial conditions 
degToRad = pi/180;
radTodeg = 180/pi;
g=9.81;

q_0 = eul2quat([0, 0, 0]*degToRad,'zyx')'; %  attitude - quaternion  
omegab_0 = [0 0 0]'; %  rad/s  angular velocity (body components)

p_0=[0 0 0]';
if traj==5
   p_0 = [-35 -2.5 0]'; % m position
end
vb_0=[0 0 0]';
omega_r_0=[0 0 0 0 0 0]'; % Initial angular speed of propellers
%% UAV inertial and dynamic parameters
% Nominal values for the parameters

UAV.Nr = 6; %number of rotors

UAV.m = 1.2; %[kg] UAV mass

UAV.J = diag([0.01 0.01 0.018]); % [kg m^2] inertia matrix

UAV.r_bg = [0 0 0]; %[m] center of mass location in the body frame
UAV.S = UAV.m*crossmat(UAV.r_bg); % [kg m] static moment

%% Linear Aerodynamics
UAV.D_tauomega = diag([0.48 0.48 2.37]); %  angular velocity damping 
UAV.D_fv =  diag([0.055 0.055 0.022]); % linear velocity damping
UAV.D=[UAV.D_fv zeros(3,3);zeros(3,3) UAV.D_tauomega]; 
UAV.M = [UAV.m*eye(3) UAV.S'; % generalized mass matrix
         UAV.S UAV.J];

UAV.Minv = inv(UAV.M); % inverse of mass matrix

%% Package properties - cubic package of side l_box and mass m_box
l_box=0.2; %[m]

r_c=[x_c y_c z_c]'; %[m]

J_box=1/6*m_box*l_box^2*eye(3); % Inertia moment of the box with respect to its own CoM axes
S=m_box*[0 -z_c y_c; z_c 0 -x_c; -y_c x_c 0]; % Static moment with respect to the CoM of the drone
J_box= J_box+m_box*[y_c^2+z_c^2 -x_c*y_c -x_c*z_c; 
                  -y_c*x_c x_c^2+z_c^2 -y_c*z_c; 
                  -z_c*x_c -z_c*y_c x_c^2+y_c^2]; % Inertia moment of the box wrt to drone CoM axis (Steiner terms)

M_box=[m_box*eye(3) S'; S J_box]; 
M_tot=UAV.M+M_box;
inv_M_tot=inv(M_tot);

%% Propellers 
UAV.b = 0.215; %[m] arm length (ell in the slides)
lambda_r_0=lambda_r*ones(6,1);
UAV.Omega_max = 10300*2*pi/60; %[rad/s] max spinning rate
UAV.Omega_min = 1260*2*pi/60; %[rad/s] min spinning rate

UAV.k_m = 1/0.05;
UAV.k_m_vec=ones(6,1)*UAV.k_m;% [s-1] Inverse of the time constant of the propeller motors
k_f = 3.65e-6;                % [N/rad^2/s^2] Thrust characteristic coeff
sigma = 0.09;      % [m] Torque-to-thrust ratio


%% Input map
b=UAV.b;      %distance from the rotors to the center of the drone
e=[0,0,1]';
PAR.e_3=e; 
%where thrust points
chi=[-1 1 -1 1 -1 1];    %propeller params


gamma=linspace(0,5*pi/3,6);                       %angles of the rotors
position=b*[cos(gamma); sin(gamma); zeros(1,6)];  %position of rotors

position_matrix=zeros(3,3,6);
for i=1:6
    position_matrix(:,:,i)=crossmat(position(:,i));  %Transform position vector into matrix
end

F_low=zeros(3,6);

for i=1:6
    F_low(:,i)=(position_matrix(:,:,i)-chi(i)*sigma*eye(3))*e; 
end
F=[e e e e e e; F_low]; %Final input map matrix

%% Trajectory parameters
omega_t=0.1;
radius=4;
w_t=1;


%% Trimming for hovering (Task 1.2) Obtained by equalazing total thrust to weight
omega_cmd_trim_1=sqrt(UAV.m*g/(UAV.Nr*k_f))*[1 1 1 1 1 1]';
omega_cmd_trim_2=zeros(6,1);
if trim_flag==1
    %Initial values
    p_0=[0 0 1]';
    omega_r_0=omega_cmd_trim_1;
end
if trim_flag==2
    theta_trim_2=atan2(UAV.D(1,1)*2,UAV.m*g);
    T_rotor=(UAV.m*g*cos(theta_trim_2)+UAV.D(3,3)*2*sin(theta_trim_2))/UAV.Nr;
    omega_cmd_trim_2=sqrt(T_rotor/k_f)*[1 1 1 1 1 1]';
    % Initial values
    q_0=eul2quat([0,theta_trim_2,0],'ZYX')';
    omega_r_0=omega_cmd_trim_2;
    vb_0=2*[cos(theta_trim_2) 0 sin(theta_trim_2)]';
end

%% Gains of the position control - LQR
A=[0 1 0; 0 0 0 ; 1 0 0 ]; 
B=[0 1 0]'; 
Q=[5 0 0;0 0.5 0; 0 0 1]; 
R=1; 
[K,~,~]=lqr(A,B,Q,R);

k_p_pos_control=K(1);
k_v_pos_control=K(2);
k_i_pos_control=K(3); 

%% Attitude controller tuning
w_n_att=12;
xi_att=0.5;

K_w_x=2*xi_att*w_n_att*UAV.J(1,1);
K_w_y=2*xi_att*w_n_att*UAV.J(2,2);
K_w_z=2*xi_att*w_n_att*UAV.J(3,3);

K_r_x=w_n_att^2*UAV.J(1,1);
K_r_y=w_n_att^2*UAV.J(2,2);
K_r_z=w_n_att^2*UAV.J(3,3);
K_r_vect=[K_r_x K_r_y K_r_z]';
A_attitude=[0 1 1; 1 0 1; 1 1 0];
k_vect=A_attitude\K_r_vect;

k1=k_vect(1);
k2=k_vect(2);
k3=k_vect(3);
k_R=[k1 k2 k3]';
k_omega=diag([K_w_x,K_w_y,K_w_z]);

%% PBMRAC

Ap=zeros(3);
Bp=eye(3);

% Gains of the adaptive control
L=1; % Predictor gain
Gamma_a=0.001*eye(6); % Learning gain

%% Simulation

simout=sim("DiMauro_Galluzzi_Andres_2024_finalversion.slx");


%% plots 
set(groot, 'defaultAxesFontSize', 14);
figure(1)
hold on
grid on
view(3)   
if traj==5
    addpath map_tools
    
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
    
    % inflate the map with the drone radius
     inflate(map3D,r)
     show(map3D)
    
     % Starting position
     startState = [-35, -2.5, 0];
     % Target position
     goalState = [32.5, 15, 0];
     % Plot map, start pose, and goal pose
     show(map3D)
     hold on
     scatter3(startState(1),startState(2),startState(3),30,".r")
     scatter3(goalState(1),goalState(2),goalState(3),30,".g")
 end
trajectory_points = squeeze(simout.trajectory.data);
x_t = trajectory_points(1,:)';
y_t = trajectory_points(2,:)';
z_t = trajectory_points(3,:)';

p1 = plot3(x_t, y_t,z_t,LineWidth=2);  % trajectory

body_point=squeeze(simout.body_position.data)';
x_b =body_point(1,:)' ;
y_b =body_point(2,:)' ;
z_b =body_point(3,:)';

p2 = plot3(x_b, y_b, z_b,LineWidth=2);  % body position


xlabel('X')
ylabel('Y')
zlabel('Z')
legend([p1 p2], {'Trajectory','Body position'})

figure(2)
time=simout.body_position.time';
plot(time,x_b,time,y_b,time,z_b); 
hold on 
plot(time,x_t,time,y_t,time,z_t); 
legend ('x UAV','y UAV','z UAV','x desired','y desired','z desired')
ylim([-10 10]); 
axis tight 
grid on 
xlabel('Time [s]')
ylabel('Position [m]')

figure(3)
velocity_body=squeeze(simout.V_body.data)';
velocity_traj=squeeze(simout.velocity_trajectory.data);

plot(time,velocity_body(1,:),time,velocity_body(2,:),time,velocity_body(3,:)); 
hold on 
plot(time,velocity_traj(1,:),time,velocity_traj(2,:),time,velocity_traj(3,:)); 
legend ('v_x UAV','v_y UAV','v_z UAV','v_x desired','v_y desired','v_z desired')
ylim([-10 10]); 
axis tight 
grid on 
xlabel('Time [s]')
ylabel('Velocity [m/s]')

figure(4)
ang=rad2deg(squeeze(simout.euler.data)); 
plot(simout.euler.time,ang); 
legend('yaw \psi','pitch \theta',  'roll \phi')
xlabel('Time [s]')
ylabel('Euler angles [º]')
grid on

% omega_cmd=squeeze(simout.omega_cmd.data);
drag=squeeze(simout.drag.data);
rot_m=squeeze(simout.rot_m.data);


figure(9)
plot(simout.euler.time,drag(1:3,:))
xlabel('Time [s]')
ylabel('Drag in inertial frame [N]')
legend('Drag in x','Drag in y', 'Drag in z','Location','best')
grid on

figure(10)
plot(simout.euler.time,[x_b-x_t,y_b-y_t,z_b-z_t])
xlabel('Time [s]')
ylabel('Error in position [m]')
legend('Error in x','Error in y', 'Error in z','Location','best')
grid on 

figure(11)
plot(time,[velocity_body(1,:)-velocity_traj(1,:);velocity_body(2,:)-velocity_traj(2,:);velocity_body(3,:)-velocity_traj(3,:)])
xlabel('Time [s]')
ylabel('Error in velocity [m/s]')
legend('Error in x','Error in y', 'Error in z','Location','best')
grid on

if flag_adaptive==1
    u_adapt=squeeze(simout.u_adapt.data);
    lambda_estimated=squeeze(simout.lambda_estimated.data);
    theta_estimated=squeeze(simout.theta_estimated.data);
    for i=1:length(time)
     lambda_norm(i)=norm(lambda_estimated(i,:))/sqrt(3);
    end
    figure(7)
    plot(simout.euler.time,[lambda_estimated,lambda_norm'])
    xlabel('Time [s]')
    ylabel('\Lambda_f/m [1/kg]')
    legend('X axis','Y axis', 'Z axis','Norm','Location','best')
    grid on
    
    figure(8)
    plot(simout.euler.time,u_adapt)
    xlabel('Time [s]')
    ylabel('Adaptive force input (inertial frame) [N]')
    legend('X axis','Y axis', 'Z axis','Location','best')
    grid on


    figure(6)
    plot(simout.euler.time,theta_estimated)
    xlabel('Time [s]')
    ylabel('\theta/m [N/kg]')
    legend('X axis','Y axis', 'Z axis','Location','best')
    grid on
end



 %% 
function x_cross = crossmat(x)
x_cross=[0 -x(3) x(2);
    x(3) 0 -x(1);
    -x(2) x(1) 0];
end

