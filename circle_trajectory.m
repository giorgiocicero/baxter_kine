% LOAD BAXTER MODEL

mdl_baxter % load the baxter model

joint_limits % load the baxter joint limits information

t_circle = zeros (4,4,360);

x = zeros(1,360);
y = zeros(1,360);
z = zeros(1,360);

radius = 0.2;


for t=0:1:359;
% Compute the Circle trajectory
t_circle(1,3, t+1) = 1;
t_circle(2,2, t+1) = -1;
t_circle(3,1, t+1) = 1;
t_circle(1,4, t+1) = 0.7; 
t_circle(4,4, t+1) = 1;

t_circle(2,4, t+1) = radius*cosd(t);
t_circle(3,4, t+1) = radius*sind(t);

% Store information to plot the circumference
x(1,t+1) = 0.700; 
y(1,t+1) = t_circle(2,4, t+1);
z(1,t+1) = t_circle(3,4, t+1);

end

% PLOTTING
figure
plot3(x,y,z); % plot the circumference
right.ikcon(t_circle)
right.plot(ans); %plot baxter CIRCLE trajectory
right.plot(ans); %plot baxter CIRCLE trajectory



