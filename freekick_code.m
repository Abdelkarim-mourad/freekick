%% Parameters

% Goal position from kick position (0,0)
goal_lon_pos    = 20;   % Goal longitudinal position    [m]
goal_lat_pos    = -5;   % Goal lateral position         [m]

% Initial conditions
% Ball position [x y z]

%input
x = input('donner x:');
y = input('donner y:');
ball_position   = [x y 0];
% Ball velocity

%input
ball_speed      = input('donner ball speed:');   % Ball speed                    [m/s]
theta           = input('donner theta:');   % Polar/elevation angle         [deg]
phi             = input('donner phi:');  % Azimuthal angle               [deg]


% Ball initial velocity components x, y and z
vx = ball_speed*cos(theta*pi/180)*cos(phi*pi/180);
vy = ball_speed*cos(theta*pi/180)*sin(phi*pi/180);
vz = ball_speed*sin(theta*pi/180);
% Ball velocity array
ball_velocity = [vx vy vz];
% Initial conditions
states0 = [ball_position ball_velocity];

% Simulation parameters
playback_speed = 1;                  % Speed of playback
tF      = 3;                            % Final time                    [s]
fR      = 30/playback_speed;            % Frame rate                    [fps]
dt      = 1/fR;                         % Time resolution               [s]
time    = linspace(0,tF,tF*fR);         % Time                          [s]

%% Simulation

% Initialize arrays to store the states
x   = zeros(length(time), 1);
y   = zeros(length(time), 1);
z   = zeros(length(time), 1);

% Initial conditions
x(1) = ball_position(1);
y(1) = ball_position(2);
z(1) = ball_position(3);

% Perform Euler integration
for i = 2:length(time)
    % Compute derivatives
    dstates = ball_dynamics(time(i-1), [x(i-1); y(i-1); z(i-1); vx; vy; vz]);
    
    % Update states using Euler method
    x(i) = x(i-1) + dstates(1) * dt;
    y(i) = y(i-1) + dstates(2) * dt;
    z(i) = z(i-1) + dstates(3) * dt;
end

%% Animation

figure('Position',[50 50 1280 720])

for i=1:length(time)
    subplot(3,2,1)
    cla
    hold on ; grid on ; axis equal
    set(gca,'FontName','Verdana','FontSize',12)
    plot(x(1:i),y(1:i),'r','LineWidth',2)
    plot(x(i),y(i),'bo','MarkerFaceColor','b','MarkerSize',4)
    xlabel('x [m]')
    ylabel('y [m]')
    title(strcat('2D Projection: x-y Plane (Time=',num2str(time(i),'%.3f'),' s)'))
    
    subplot(3,2,3)
    cla
    hold on ; grid on ; axis equal
    set(gca,'FontName','Verdana','FontSize',12)
    plot(y(1:i),z(1:i),'r','LineWidth',2)
    plot(y(i),z(i),'bo','MarkerFaceColor','b','MarkerSize',4)
    xlabel('y [m]')
    ylabel('z [m]')
    title('2D Projection: y-z Plane')
    
    subplot(3,2,5)
    cla
    hold on ; grid on ; axis equal
    set(gca,'FontName','Verdana','FontSize',12)
    plot(z(1:i),x(1:i),'r','LineWidth',2)
    plot(z(i),x(i),'bo','MarkerFaceColor','b','MarkerSize',4)
    xlabel('z [m]')
    ylabel('x [m]')
    title('2D Projection: z-x Plane')
    
    subplot(3,2,[2 4 6])
    cla
    hold on ; grid on ; axis equal
    set(gca,'FontName','Verdana','FontSize',12)
    plot_field([-206.4874  199.9462   79.6781],goal_lon_pos,goal_lat_pos)
    plot3(x(1:i),y(1:i),z(1:i),'r','LineWidth',2)
    plot3(x(i),y(i),z(i),'bo','MarkerFaceColor','b','MarkerSize',4)
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    title('3D Trajectory')
    
    drawnow
end

% %% Determine if it's a goal
% 
% % Define the goal area boundaries
% goal_lower_bound = -2.44/2; % Lower bound of the goal area
% goal_upper_bound = 2.44/2;  % Upper bound of the goal area
% 
% % Determine if the ball crossed the goal line within the goal area
% if y(end) >= goal_lower_bound && y(end) <= goal_upper_bound && x(end) >= goal_lon_pos
%     disp('Goal! The free kick resulted in a goal!');
% else
%     disp('The free kick did not result in a goal.');
% end

function plot_field(camera_position,goal_lon_pos,goal_lat_pos)

    % Goal
    goal_width      = 7.32; % [m]
    goal_height     = 2.44; % [m]
    % Penalty area
    penalty_area_width  = 40.2; % [m]
    penalty_area_length = 16.5; % [m]
    % Goal area
    goal_area_width  = 18.3;    %[m]
    goal_area_length = 5.5;     %[m]
    % Penalty mark
    penalty_mark_distance_from_goal = 11; %[m]
    % Penalty arc
    penalty_arc_radius      = 9.15;
%     penalty_arc_center_x    = goal_lon_pos-penalty_mark_distance_from_goal;
%     penalty_arc_center_y    = 0;
    theta_amp = acos((penalty_area_length-penalty_mark_distance_from_goal)/penalty_arc_radius);
    theta = linspace(-theta_amp+pi,theta_amp+pi,10);
    x_arc = penalty_arc_radius*cos(theta)+(goal_lon_pos-penalty_mark_distance_from_goal);
    y_arc = penalty_arc_radius*sin(theta);
    % Endline
    endline_width = 50;

    set(gca,'CameraPosition',camera_position)
    set(gca,'xlim',[-10 goal_lon_pos+5],'ylim',[-endline_width/2+goal_lat_pos endline_width/2+goal_lat_pos],'zlim',[0 5])
    %     Set the color of the field to green
    field_color = [0 0.5 0]; % Green color
    
%     Plot the field with green color
    fill3([0 goal_lon_pos goal_lon_pos 0],...
          [-endline_width/2+goal_lat_pos -endline_width/2+goal_lat_pos endline_width/2+goal_lat_pos endline_width/2+goal_lat_pos],...
          [0 0 0 0], field_color)
      
    % Goal
    plot3(  goal_lon_pos*ones(4,1),...
            [-goal_width/2+goal_lat_pos -goal_width/2+goal_lat_pos goal_width/2+goal_lat_pos goal_width/2+goal_lat_pos],...
            [0 goal_height goal_height 0],'k','LineWidth',2)
       % Penalty area
    plot3(  [goal_lon_pos goal_lon_pos-penalty_area_length goal_lon_pos-penalty_area_length goal_lon_pos],...
            [-penalty_area_width/2+goal_lat_pos -penalty_area_width/2+goal_lat_pos penalty_area_width/2+goal_lat_pos penalty_area_width/2+goal_lat_pos],...
            [0 0 0 0],'g')
    % Goal area
    plot3(  [goal_lon_pos goal_lon_pos-goal_area_length goal_lon_pos-goal_area_length goal_lon_pos],...
            [-goal_area_width/2+goal_lat_pos -goal_area_width/2+goal_lat_pos goal_area_width/2+goal_lat_pos goal_area_width/2+goal_lat_pos],...
            [0 0 0 0],'g')
    % Penalty mark
    plot3(goal_lon_pos-penalty_mark_distance_from_goal,goal_lat_pos,0,'wo','MarkerFaceColor','w','MarkerSize',2)
    % Penalty arc
    plot3(x_arc,y_arc+goal_lat_pos,zeros(length(theta)),'w')
    % Endline
    plot3([goal_lon_pos goal_lon_pos],[-endline_width/2+goal_lat_pos endline_width/2+goal_lat_pos],[0 0],'k')
    % Ball initial position
%     plot3(0, 0, 0,'wo','MarkerFaceColor','w','MarkerSize',2)
end

function dstates = ball_dynamics(~,states)
% States
% x   = states(1);
% y   = states(2);
% z   = states(3);
dx  = states(4);
dy  = states(5);
dz  = states(6);

v = sqrt(dx^2 + dy^2 + dz^2);   % Speed                     [m/s]

% Parameters
m   = 0.425;                    % Mass                      [kg]
A   = 0.0388;                   % Cross-sectional area      [m2]
g   = 9.8;                      % Gravity                   [m/s2]
rho = 1.2;                      % Air density               [kg/m3]
r   = 0.111;                    % Radius                    [m]
CM  = 1;                        % Magnus coefficient (Magnus)
CD  = 0.275;                    % Drag coefficient (Prandtl)

% Ball angular velocity
wx = 0;
wy = 0;
wz = 6*2*pi;        % rad/s

% Lumped coefficients
C1 = 1/2*CD*rho*A;
C2 = 1/2*CM*rho*A*r;

ddx = (-C1*v*dx + C2*(wy*dz - wz*dy))/m;
ddy = (-C1*v*dy + C2*(wz*dx - wx*dz))/m;
ddz = (-C1*v*dz + C2*(wx*dy - wy*dx) - m*g)/m;

dstates = [dx ; dy ; dz ; ddx ; ddy ; ddz];

end

function [position,isterminal,direction] = ball_floor_or_end(~,y,goal_lon_pos)
% Terminate simulation when ball touches the floor or crosses the goal line.
% Define the goal area boundaries
goal_lower_bound = -2.44/2; % Lower bound of the goal area
goal_upper_bound = 2.44/2;  % Upper bound of the goal area

% Check if the ball crosses the goal line within the goal area
if y(2) >= goal_lower_bound && y(2) <= goal_upper_bound && y(1) >= goal_lon_pos
    % Ball crossed the goal line within the goal area
    position = y(1) - goal_lon_pos; % Event function becomes zero when the ball crosses the goal line
    isterminal = 1; % Terminate the simulation when the event function is triggered
    direction = 0; % Event function should trigger whether the ball enters or leaves the goal area
else
    % Ball touches the floor or outside the goal area
    position = y(3); % Event function becomes zero when the ball touches the floor or outside the goal area
    isterminal = 1; % Terminate the simulation when the event function is triggered
    direction = -1; % Event function should trigger only when the ball touches the floor or leaves the goal area
end
end