function icp_function_2d_3d(icp_inputs,count)
%icp_function Summary of this function goes here
%   Detailed explanation goes here
Q = icp_inputs.Q; P = icp_inputs.P;
R = icp_inputs.R; t = icp_inputs.t;
robot_pose = icp_inputs.robot_pose;
map_r = icp_inputs.map_r; 
robot_pose_x = icp_inputs.robot_pose_x;
robot_pose_y = icp_inputs.robot_pose_y;
iteration_count = icp_inputs.iteration_count;

    R{count} = [1 0 ; 0 1 ];
    t{count} = [0 0 ]';
    P_corrected = P;
    
    iteration = 1;
    corres = correspondences(P, Q);
    err_new =sum((corres(3,:)).^2)*1000;
    err_prev =inf;
    err =inf;
    err_all = zeros(1,100);

% the while expression will keep running if:
% 1- the sum of the squre error is more than 5
% 2- the number of iteration is less than 50
% 3- error change is more than 1 unless the error is high (ex 1000)
while err > 2 && iteration <= 50 && (err_prev-err_new>1 || err_new >200)  % i change 2 to 4 because of the noise
    % Compute correspondences
    corres = correspondences(P_corrected, Q);

    % Compute cross covariance and element has cross info
    [K, P_mean, Q_mean, w] = crossCovMat_w(P_corrected,Q,corres);
    %[K, P_mean, Q_mean] = crossCovMat(P_corrected,Q,corres);

    % % Find R and t from SVD
    [U, ~, V] = svd(K);
    R_found = U*V';
    R{count} = R_found * R{count};
    t_found = Q_mean - R_found * P_mean;
    t{count} = t_found + t{count};

    %P_corrected = R_found* P_corrected + t_found;
    P_corrected = R{count}* P + t{count};


    % sum of the square error
    %err = sum((corres(3,:)).^2);
    err = 0;
    for i_w = 1:size(corres,2)
        err = err + corres(3,i_w)^2 * w(i_w);
%         err = err + corres(3,i_w).^2;
    end
%     err_all(iteration) = err;
%     err_prev = err_new;
%     err_new = err;
    iteration = iteration + 1;
end

%err_all_scans{count} = err_all;
%err_final_scans(1,count) = iteration-1;
%err_final_scans(2,count) = err;

robot_pose{count+1} = R{count} * robot_pose{1} + t{count};

map_r{count+1} = P_corrected;

% the next loop to mach with the first scan (initial)
if count>1
    prev_count = count-1;
    for ii = 1:count-1
        
        map_r{count+1} = R{prev_count}* map_r{count+1} + t{prev_count};

        robot_pose{count+1} = R{prev_count} * robot_pose{count+1} + t{prev_count};

        prev_count = prev_count-1;
    end
end

iteration_count(count) = iteration - 1;
assignin('base','iteration_count',iteration_count)

robot_pose_x(count+1) = robot_pose{count+1}(1,1);
robot_pose_y(count+1) = robot_pose{count+1}(2,1);

assignin('base','map_r',map_r)

assignin('base','R',R)
assignin('base','t',t)
assignin('base','robot_pose',robot_pose)
assignin('base','robot_pose_x',robot_pose_x)
assignin('base','robot_pose_y',robot_pose_y)
assignin('base','err',err)
assignin('base','iteration',iteration)

end

% For each point in P find closest one in Q.
function corres = correspondences(P, Q)
    p_size = length(P);
    q_size = length(Q);
    corres = zeros(3,p_size);
    for i = 1:p_size
        p_point = P(:,i);
        min_dist = inf;
        
        for j = 1:q_size
            q_point = Q(:,j);
            dist = norm (q_point-p_point);
            
            if dist < min_dist
                min_dist = dist;
                chosen_idx = j;                
            end
        end
        corres(:,i)= [i,chosen_idx,min_dist]';
    end
    % next I kept just close corres if more than one chosen
    for i = 1: size(corres,2)
        for j = 1 : size(corres,2)
           if i~=j && corres(2,i)== corres(2,j) && corres(3,i)<=corres(3,j) && corres(3,i)~=0
                corres(3,j) = nan;
           end
        end
    end
    corres = corres(:,all(~isnan(corres)));  
end

% Compute cross covariance
function [K, P_mean, Q_mean,w] = crossCovMat_w(P,Q,corres)
    K = zeros(2,2);
    w_max = 40; % <<<<<<<<<<<<<<<<<=============wieght max dis============
    P_mean = zeros(2,1);
    Q_mean = zeros(2,1);
    w =zeros(size(corres(1,:)));
    for i = 1 : size(corres,2)
        p_indx = corres(1,i);
        q_indx = corres(2,i);
        p_v = sqrt( (Q(1,q_indx))^2 +  (Q(2,q_indx))^2 );
        if p_v > w_max
            w(i) =0;
        else
            w(i) = (w_max - p_v)/w_max;
        end
        
        Q_mean = Q_mean + Q(:,q_indx) * w(i);
        P_mean = P_mean + P(:,p_indx) * w(i);
    end
    Q_mean = Q_mean/sum(w);
    P_mean = P_mean/sum(w);
    
    for i = 1 : size(corres,2)
        p_indx = corres(1,i);
        q_indx = corres(2,i);
        
        K = K + (Q(:,q_indx) -Q_mean)* (P(:,p_indx)-P_mean)';
        
    end

end
