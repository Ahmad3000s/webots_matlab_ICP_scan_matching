% MATLAB controller for Webots
% File:          drone_matlab.m
% Date:          20/04/2021
% Description:   Indoor mapping for stockpile volume estimation in cement plant
% Author:        Ahmad Alsayed
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

timestep = 32;

% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');
% LED
flLed = wb_robot_get_device('front left led');
frLed = wb_robot_get_device('front right led');
% Propeller
flp = wb_robot_get_device('front left propeller');
frp = wb_robot_get_device('front right propeller');
rlp = wb_robot_get_device('rear left propeller');
rrp = wb_robot_get_device('rear right propeller');
%IMU
imu = wb_robot_get_device('inertial unit');
wb_inertial_unit_enable(imu,timestep);
% Gyro
gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro,timestep);
% compass
compass = wb_robot_get_device('compass');
wb_compass_enable(compass,timestep);
% GPS
gps = wb_robot_get_device('gps');
wb_gps_enable(gps,timestep);
% lidar
lidar = wb_robot_get_device('lidar');
wb_lidar_enable(lidar,timestep);
wb_lidar_enable_point_cloud(lidar);



% Set propeller initial speed
wb_motor_set_position(flp, inf);
wb_motor_set_velocity(flp, 1.0);
wb_motor_set_position(frp, inf);
wb_motor_set_velocity(frp, -1.0);
wb_motor_set_position(rlp, inf);
wb_motor_set_velocity(rlp, -1.0);
wb_motor_set_position(rrp, inf);
wb_motor_set_velocity(rrp, 1.0);

%#### define some virables ###
var_init()

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
step = 0; % step for each loop
sample = 0; % for low frequancy plotting
count = 0; % for counting the icp variables
count_pc = 1; % lidar scans count

navigate_f = true; % false OR true
icp_fun = true;

nav_yaw_error_prev = 0;
pc_R = 0; count_x = 0;
pc_F = 0; count_z = 0;

time_nav = zeros(1,10000); nav_yaw_error = time_nav;
Int_c =time_nav;
scan = cell(1,100000);
scan_all = scan;
pose = zeros(10000,3);
gps_imu_all = zeros(100000,7);

scan_all_1 = cell(1,500);
scan_all_2 = scan_all_1;

figure
while wb_robot_step(timestep) ~= -1
    step = step+1;
    time = wb_robot_get_time();
    % read the sensors, e.g.:
    % rgb = wb_camera_get_image(camera);

    % Retrieve robot position and attitude using gps,imu,gyro.
    gps_log = wb_gps_get_values(gps); % array [x,y,z]
    x = gps_log(1); y = gps_log(2); z = gps_log(3);
    altitude = y;
    gyro_log = wb_gyro_get_values(gyro);
    roll_acceleration = gyro_log(1);
    pitch_acceleration = gyro_log(2);
    yaw_acceleration = gyro_log(3);
    imu_log = wb_inertial_unit_get_roll_pitch_yaw(imu);       
    roll = imu_log(1); pitch = imu_log(2); yaw = imu_log(3);
    % save to files
%     writematrix([time gps_log imu_log],...
%     fulldestination_gps_imu,'WriteMode','append')
    gps_imu_all(step,:) = [time gps_log imu_log];
    
    % get lidar scans
    PC_3D_0 = wb_lidar_get_layer_point_cloud(lidar, 0);
    PC_3D_1 = wb_lidar_get_layer_point_cloud(lidar, 1);
    PC_3D_2 = wb_lidar_get_layer_point_cloud(lidar, 2);
    PC_3D_3 = wb_lidar_get_layer_point_cloud(lidar, 3);
    PC_3D_4 = wb_lidar_get_layer_point_cloud(lidar, 4);
    
    for point_id = 1:512
        x_0(point_id) = PC_3D_0(point_id).x;
        y_0(point_id) = PC_3D_0(point_id).y;
        z_0(point_id) = PC_3D_0(point_id).z;
    end  
    for point_id = 1:512
        x_0(point_id+512) = PC_3D_1(point_id).x;
        y_0(point_id+512) = PC_3D_1(point_id).y;
        z_0(point_id+512) = PC_3D_1(point_id).z;
    end
    for point_id = 1:512
        x_0(point_id+512*2) = PC_3D_2(point_id).x;
        y_0(point_id+512*2) = PC_3D_2(point_id).y;
        z_0(point_id+512*2) = PC_3D_2(point_id).z;
    end 
    for point_id = 1:512
        x_0(point_id+512*3) = PC_3D_3(point_id).x;
        y_0(point_id+512*3) = PC_3D_3(point_id).y;
        z_0(point_id+512*3) = PC_3D_3(point_id).z;
    end         
    for point_id = 1:512
        x_0(point_id+512*4) = PC_3D_4(point_id).x;
        y_0(point_id+512*4) = PC_3D_4(point_id).y;
        z_0(point_id+512*4) = PC_3D_4(point_id).z;
    end
    scan_id(1:512*5,1) = step;
    scan_all{step}=[scan_id x_0' y_0' z_0'];

    
    PC = PC_3D_4; % from 0 down to layer-1
    range = wb_lidar_get_layer_range_image(lidar, 4);
    
    pc_x = zeros(1,512); pc_z=pc_x;
    for i_pc=1:512
        pc_x(i_pc) = PC(i_pc).x;
        pc_z(i_pc) = PC(i_pc).z;
    end
    
    if navigate_f == true && time > 12

        % any(range(129+25:257)< 10)
        if any(range(256-32:257+32)< 8) %10 %any(range(384+25:511)< 10) %pc_F < 10  % trun left
            yaw_disturbance = -0.15; % turn left
            pitch_disturbance = 0.50; % move slow forword
        else
            desired_side_dis = 4; %6

            time_nav(step) = time;
            for ii = 128:128+25%384:384+25%384-25:384+25
                count_x=count_x+1;
                pc_R = pc_R + PC(ii).x;
            end
            pc_R = abs(pc_R/count_x);
            count_x = 0;           

            kp_yaw_disturbance = 0.05;
            kd_yaw_disturbance = 1.0;
            ki_yaw_disturbance = 0.01;

            nav_yaw_error(step) = desired_side_dis - pc_R;
            Prop = kp_yaw_disturbance * nav_yaw_error(step);
            Int_c(step)  = (nav_yaw_error(step) + nav_yaw_error(step-1))*0.032/2; % integration of the error
            % I_c = ki_yaw_disturbance * sum(Int_c); % the sum of the integration of the error
            I_c = 0;
            Der = (nav_yaw_error(step) - nav_yaw_error(step-1))/0.032; %dt=32ms
            yaw_disturbance = -(Prop + I_c + Der);
            yaw_disturbance = (max(-0.15, min(0.15, yaw_disturbance)));
            pitch_disturbance = 0.50;

        end
    end

    roll = roll + 3.141592653589793 /2;

    roll_input  = k_roll_p  * (max(-1, min(1, roll))) + roll_disturbance + roll_acceleration;
    pitch_input = k_pitch_p * (max(-1, min(1, pitch)))  + pitch_disturbance -  pitch_acceleration;
    yaw_input   = yaw_disturbance;

    if target_altitude < 0;target_altitude = 0;end
    clamped_difference_altitude = target_altitude - y;  
    if sign(clamped_difference_altitude) ~= sign(sum_v);sum_v = 0.0;end
    sum_v = sum_v + clamped_difference_altitude; 

    vertical_input = k_vertical_thrust + k_vertical_p * (max(-0.05, min(0.05, clamped_difference_altitude))) + k_vertical_offset * sum_v;

    front_left_motor_input =  vertical_input - roll_input - pitch_input + yaw_input;
    front_right_motor_input =  vertical_input + roll_input - pitch_input - yaw_input;
    rear_left_motor_input =  vertical_input - roll_input + pitch_input - yaw_input;
    rear_right_motor_input =  vertical_input + roll_input + pitch_input + yaw_input;

    % send actuator commands, e.g.:
    %  wb_motor_set_postion(motor, 10.0);
    wb_motor_set_velocity(flp,front_left_motor_input);
    wb_motor_set_velocity(frp,-front_right_motor_input);
    wb_motor_set_velocity(rlp,-rear_left_motor_input);
    wb_motor_set_velocity(rrp,rear_right_motor_input);  

    % icp slam
    sample = sample + 1;
    if sample > 100 && time > 12 && icp_fun == true

        scan{count_pc} = [pc_x; pc_z];

        %%%%%%%%  ICP  %%%%%%%%%%%%%%%%%
        if count == 0
            map_r{1} = scan{1};
        else
            
            Q = scan{count_pc-1};
            P = scan{count_pc};

            icp_inputs.Q =Q; icp_inputs.P =P;
            icp_inputs.R =R; icp_inputs.t =t;
            icp_inputs.robot_pose =robot_pose;
            icp_inputs.map_r = map_r;
            icp_inputs.robot_pose_x = robot_pose_x;
            icp_inputs.robot_pose_y = robot_pose_y;

            icp_function(icp_inputs,count);
            wb_console_print('Error = ', WB_STDOUT);
            wb_console_print(num2str(err), WB_STDOUT);   % disp error 
            wb_console_print('iteration = ', WB_STDOUT);
            wb_console_print(num2str(iteration), WB_STDOUT);   % disp error 

            
            if err > 100
                disp('high error')
                disp(err)
                break
            end
        end
        %%%%%%%%%%% END ICP %%%%%%%%%%%%%%

        %%%%%%%%%% occupancyMap %%%%%%%%%%
        
        if count == 0
            R_occ = [1 0; 0 1];
        else
            R_occ = R_occ * R{count};
        end
        
        scan_rotate = R_occ*[pc_x; pc_z];
        scan_rotate=scan_rotate(:,all(~isinf(scan_rotate)));
        scan_rotate=scan_rotate(:,all(~isnan(scan_rotate)));
        x_oc = (scan_rotate(1,:));
        y_oc = (scan_rotate(2,:));
        [theta,rho] = cart2pol(x_oc,y_oc);
        scan_oc = lidarScan(rho,theta);

        pose(count_pc,:) = pose_o + [robot_pose_x(count+1),robot_pose_y(count+1) ,0 ];

        insertRay(map,pose(count+1,:),scan_oc,maxrange);
        subplot(1,2,1)
        show(map); hold on
        plot(pose(1:count_pc,1),pose(1:count_pc,2),'b-o'); hold off % pose(1:count_pc,1)
        %%%%%%%%%% end occupancyMap %%%%%%%%

        subplot(1,2,2)
        scatter(pc_x,pc_z,'black',".")
        axis equal; xlabel('x [m]'); ylabel('y [m]')
        xlim([min(pc_x)-5 max(pc_x)+5]); ylim([min(pc_z)-5 max(pc_z)+5])  
        title('lidar')

        sample = 0;
        count = count +1;
        count_pc = count_pc+1;
    end


    drawnow;

    % save gps & lidar data
    

    if time > 240 %460
%         % gather
%         scan_all = [scan_all_1, scan_all_2, scan_all_3, scan_all_4, scan_all_5];
        save(destdirectory_scan_1,'scan_all')
        save(fulldestination_gps_imu,'gps_imu_all')
        
        break
    end
end

% cleanup code goes here: write data to files, etc.

disp('Done')
