function var_init()
%var_init Summary of this function goes here
%   Detailed explanation goes here

%# PID parameter for drone stability
assignin('base','k_vertical_thrust',68.5) %#90FOR Physical # with this thrust, the drone lifts.
assignin('base','k_vertical_offset',0.001)   %# Vertical offset where the robot actually targets to stabilize itself.
assignin('base','k_vertical_p',40)       %# P constant of the vertical PID.
assignin('base','k_roll_p',30) %50        # P constant of the roll PID.
assignin('base','k_pitch_p',30)  %5       # P constant of the pitch PID.

assignin('base','target_altitude',7)
assignin('base','yaw_disturbance',0.0) %0
assignin('base','yaw_TARGET',90)
assignin('base','sum_v',0.0)
assignin('base','pitch_disturbance',0.5)    %0;  % move streaght
assignin('base','roll_disturbance',0)


% varibales to save and plot
assignin('base','scan',cell(1,100))

assignin('base','R',cell(1,40))
assignin('base','t',cell(1,40))

assignin('base','map_r',cell(1,40))

map = occupancyMap(40,50,10);
assignin('base','map',map)
assignin('base','maxrange',40)
assignin('base','pose_o',[10,40,0])
assignin('base','pose',[])


robot_pose = cell(1,100);
robot_pose_x(1:100) = NaN;
robot_pose_y = robot_pose_x;

robot_pose{1} = [0;0];
robot_pose_x(1) = robot_pose{1}(1,1);
robot_pose_y(1) = robot_pose{1}(2,1);

assignin('base','robot_pose',robot_pose)
assignin('base','robot_pose_x',robot_pose_x)
assignin('base','robot_pose_y',robot_pose_y)

% generate file names for saving:
destdirectory = fullfile('saved_data',sprintf('save_data %s', datestr(now,'mm-dd-yyyy HH-MM-SS')));
mkdir(destdirectory); 
fulldestination_gps_imu = fullfile(destdirectory, 'gps_imu.mat');
% writematrix(["timestamp" "x" "y" "z" "roll" "pitch" "yaw"],...
%     fulldestination_gps_imu,'WriteMode','append')
assignin('base','fulldestination_gps_imu',fulldestination_gps_imu)

% save lidar and make folder
% destdirectory_scan = fullfile(destdirectory,'\scan');
% mkdir(destdirectory_scan); 
destdirectory_scan_1 = fullfile(destdirectory,'PC_3D_all_1.mat');
assignin('base','destdirectory_scan_1',destdirectory_scan_1)




end

