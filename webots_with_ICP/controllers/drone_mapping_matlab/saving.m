%saving
% writematrix(A,'M.xls')
% writematrix(A*2,'M.xls','WriteMode','append')
clear
destdirectory = fullfile('saved_data',sprintf('save_data %s', datestr(now,'mm-dd-yyyy HH-MM-SS')));
mkdir(destdirectory); 

% save lidar and make folder
destdirectory_scan = fullfile(destdirectory,'\scan');
mkdir(destdirectory_scan); 
destdirectory_scan = fullfile(destdirectory_scan,'PC_3D_all.mat');


%%
for i = 1:10
    GPS = rand(1,3);
    fulldestination = fullfile(destdirectory, 'gps.xlsx');
    writematrix(GPS,fulldestination,'WriteMode','append')
    PC_3D{i} = [1 2 3];
    save(destdirectory_scan,'PC_3D')
    
    
end

%%
s = scan_all{1, 600};
scatter3(s(:,2),s(:,4),-s(:,3))  
axis equal
%%
eval(['NAME_' num2str(i) '=dataset;'])
%%
for step = 1:12000
    if step < 2001
        scan_all_1{step}=[1 2 3 4];
    elseif step < 4001 && step > 2000
        scan_all_2{step-2000}=[1 2 3 4];
    elseif step < 6001 && step > 4000
        scan_all_3{step-4000}=[1 2 3 4];   
    elseif step < 8001 && step > 6000
        scan_all_4{step-6000}=[1 2 3 4];
    else
        scan_all_5{step-8000}=[1 2 3 4];    
    end
end
% gather
scan_all = [scan_all_1, scan_all_2, scan_all_3];
%%

clear
