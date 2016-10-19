
mdl_baxter;

joint_limits;

load('Q_to_test_3.mat');

load('T_original.mat');
load('T_calculated_ikcon.mat');
load('T_calculated_ikunc.mat');
load('q_ikunc.mat');
load('q_ikcon.mat');


% if ~exist('T_original')
%     T_original = right.fkine(joints_3_r);
% end
% 
% if ~exist('q_ikunc')
%     q_ikunc = right.ikunc(T_original);
% end
% 
% if ~exist('q_ikcon')
%     q_ikcon = right.ikcon(T_original);
% end

[q_opt, q_opt_error] = optimizationFunction(right, q_ikunc, joints_3_r, q_ikcon);

% if ~exist('T_calculated_ikunc')
%     T_calculated_ikunc = right.fkine(q_ikunc);
% end
% 
% if ~exist('T_calculated_ikcon')
%     T_calculated_ikcon = right.fkine(q_ikcon);
% end
% 

T_opt = right.fkine(q_opt);

sim_size_ = size(joints_3_r(:,1));
sim_size = sim_size_(1,1);

error_norm_ikunc = zeros(1, sim_size);
error_norm_opt = zeros(1, sim_size);
error_norm_opt_isolated = zeros(1, sim_size);

x = zeros(1,sim_size);
y = zeros(1,sim_size);
z = zeros(1,sim_size);
x_ikcon = zeros(1,sim_size);
y_ikcon = zeros(1,sim_size);
z_ikcon = zeros(1,sim_size);
x_ikunc = zeros(1,sim_size);
y_ikunc = zeros(1,sim_size);
z_ikunc = zeros(1,sim_size);
real_x = zeros(1,sim_size);
real_y = zeros(1,sim_size);
real_z = zeros(1,sim_size);

for i=1:sim_size
    error_norm_ikunc(1, i) = norm(T_original(:,:,i)) - norm(T_calculated_ikunc(:,:,i));
    error_norm_opt(1, i) = norm(T_original(:,:,i)) - norm(T_opt(:,:,i));
    error_norm_opt_isolated(1, i) = norm(T_opt(:,:,i)) - norm(T_calculated_ikunc(:,:,i));
    
    x(1,i) = T_original(1,4,i);
    y(1,i) = T_original(2,4,i);
    z(1,i) = T_original(3,4,i);
    
    x_ikcon(1,i) = T_calculated_ikcon(1,4,i);
    y_ikcon(1,i) = T_calculated_ikcon(2,4,i);
    z_ikcon(1,i) = T_calculated_ikcon(3,4,i);
    
    x_ikunc(1,i) = T_calculated_ikunc(1,4,i);
    y_ikunc(1,i) = T_calculated_ikunc(2,4,i);
    z_ikunc(1,i) = T_calculated_ikunc(3,4,i);
    
    real_x(1,i) = T_opt(1,4,i);
    real_y(1,i) = T_opt(2,4,i);
    real_z(1,i) = T_opt(3,4,i);
end

is_limit_original = zeros(1, sim_size);
is_limit_ikcon = zeros(1, sim_size);
is_limit_ikunc = zeros(1, sim_size);
is_limit_opt_ikunc = zeros(1, sim_size);

for i=1:sim_size
    res = right.islimit(joints_3_r(i,:));
    is_limit_original(1, i) = norm(res(:,1)) + norm(res(:,2));
    
    res = right.islimit(q_ikcon(i,:));
    is_limit_ikcon(1, i) = norm(res(:,1)) + norm(res(:,2));
    
    res = right.islimit(q_ikunc(i,:));
    is_limit_ikunc(1, i) = norm(res(:,1)) + norm(res(:,2));
    
    res = right.islimit(q_opt(i,:));
    is_limit_opt_ikunc(1, i) = norm(res(:,1)) + norm(res(:,2));
end

feasible_result_original = norm(is_limit_original(1,:));
feasible_result_ikcon = norm(is_limit_ikcon(1,:));
feasible_result_ikunc = norm(is_limit_ikunc(1,:));
feasible_result_opt = norm(is_limit_opt_ikunc(1,:));


joint_variation_original = zeros(1, sim_size);
joint_variation_opt = zeros (1,sim_size);

for i=10:sim_size-1
   joint_variation_original(1,i) = norm(joints_3_r(i,:) - joints_3_r(i+1,:)); 
   joint_variation_opt(1,i) = norm(q_opt(i,:) - q_opt(i+1,:)); 
end

%% PLOTTING ORIGINAL

figure

answer = input('Do you want to plot the original movements? (0/1)');
if(answer == 1)
    pause();
    hold on;
    plot3(x,y,z,'color','green');
    pause();
    hold off;
    right.plot(joints_3_r,'fps', 20);
    clf
end
%% PLOTTING RESULT WITHOUT OPTIMIZATION BY USING IKCON
answer = input('Do you want to plot the movements obtained by using IKCON function of the tool? (0/1)');
if(answer == 1)
    pause();
    hold on;
    plot3(x,y,z,'color','green');
    pause();
    plot3(x_ikcon,y_ikcon,y_ikcon,'color','red');
    pause();
    hold off;
    right.plot(q_ikcon,'fps', 20);
    clf
end

%% PLOTTING RESULT WITHOUT OPTIMIZATION BY USING IKUNC
answer = input('Do you want to plot the movements obtained by using IKUNC function of the tool? (0/1)');
if(answer == 1)
    pause();
    hold on;
    plot3(x,y,z,'color','green');
    pause();
    plot3(x_ikunc,y_ikunc,z_ikunc,'color','red');
    pause();
    hold off;
    right.plot(q_ikunc,'fps', 20);
    clf
end

%% PLOTTING FINAL RESULT WITH OPTIMIZATION CHOSEN
SDisplay = sprintf('Plotting the final movements obtained by using IKUNC and with the optimization algorithm applied');
disp(SDisplay);
pause();
hold on;
plot3(x,y,z,'color','green');
pause();
plot3(real_x,real_y,real_z,'color','red');
pause();
hold off;
right.plot(q_opt,'fps', 20);





%%
clear is_limit_original;
clear is_limit_ikcon;
clear is_limit_ikunc;
clear is_limit_opt_ikunc;
clear i;
clear answer;
clear joints_3_r;
clear res;
clear real_x;
clear real_y;
clear real_z;
clear SDisplay;
clear x;
clear x_ikcon;
clear y_ikcon;
clear y;
clear z_ikcon;
clear z;
clear x_ikunc;
clear y_ikunc;
clear z_ikunc;
clear sim_size;
clear sim_size_;
