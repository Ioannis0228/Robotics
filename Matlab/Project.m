% Ioannis Orthodoxou 
% AEM: 10822

clc; clear;close all;

% Data
l = 1;
l_0 = 0.1;
h = 0.7;

% Homogeneous Transformation for the base of room
G_0 = eye(4);

% Homogeneous Tranformation Door frame relative to base frame
G_0D = transl([0,2,0]);
% G_0D = [1 0 0 0;
%        0 1 0 2;
%        0 0 1 0;
%        0 0 0 1];

% Homogeneous Transformation Knob frame relative to Door frame
G_DH = [0 1 0 l-l_0;
        -1 0 0 0;
        0 0 1 h;
        0 0 0 1];
            
% Homogeneous Tranformation Knob frame relative to base frame
G_0H = G_0D * G_DH;

% Homogeneous Tranformation Robot End effector relative to Knob frame 
G_HE = [0 0 -1 0.1;
        0 1 0 0.1
        1 0 0 0
        0 0 0 1];

% Homogeneous Tranformation Base of robot relative to base frame
G_0B = transl([1 1 0]);

%% PART A
% Times
t0 = 0; t1 = 1.5; t2 = 3.5; tf = 5;
dt = 0.01;

%t_all = t0:dt:tf;

% Angles for the first move
theta0_1 = 0;
thetaf_1 = -45 * pi/180;

%view(30,30);

g_T = cell(1);

%% First Move

% Rotation Trajectory of knob using 5 degree polynomial
theta_firstmove = mytrajectory(theta0_1, thetaf_1, 0:dt:1.5);

i = 1;
for t = t0:dt:t1 
    % Calculate the Homogeneous Tranformation Knob relative to base frame for each dt
    T = G_0H * trotx(theta_firstmove(i)); 

    g_T{i} = T; % Save the Matrix to use it for plots and later in Part B
    i = i+1;

    % % CREATE 3D plot with the position and rotation of each frame (SLOW)
    % trplot(G_0,'axis',[0 2 0 2 0 2],'frame','0', 'color', 'k', 'length', 0.4);
    % hold on;
    % trplot(G_0D,'color', 'b', 'frame', 'D', 'length', 0.4);
    % trplot(G_0H,'color', 'r', 'frame', 'H', 'length', 0.4);
    % trplot(T, 'frame', 'H', 'color', 'g', 'length', 0.4);
    % title(sprintf('t = %.2f sec', t));
    % drawnow;
    % hold off;


    % % CREATE GIF 
    % frame = getframe(gcf);
    % im = frame2im(frame);
    % [A, map] = rgb2ind(im, 256);
    % 
    % % Save to GIF
    % if t == 0
    %     imwrite(A, map, 'animation.gif', 'gif', 'LoopCount', Inf, 'DelayTime', 0.05);
    % else
    %     imwrite(A, map, 'animation.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    % end
end

% Homogeneous Transformation Knob to door frame at the end of the first move
G_DH_last = G_DH * trotx(theta_firstmove(i-1));
% return
%% Second Move

% Angles for the second move
theta0_2 = 0;
thetaf_2 = -30 * pi/180;

% Rotation Trajectory of door frame using 5 degree polynomial
% Start from dt because 0 is the t0=1.5 and we have the transformation from
% the previous move as tf=1.5
% If we start at 1.5 the transformation is already in the g_T and will end up with 2 same transformations 
theta2 = mytrajectory(theta0_2, thetaf_2,dt:dt:2);
i = 1;

for t = t1+dt:dt:t2 
    
    % Calculate the Homogeneous Tranformation door relative to base frame for each dt
    T = G_0D * trotz(theta2(i)); 

    % Save the Homogeneous Tranformation knob relative to base frame to use it for plots and later in Part B
    g_T{(t1/dt) + i +1} = T * G_DH_last; 
    i = i + 1 ;

    % % CREATE 3D plot with the position and rotation of each frame (SLOW)
    % trplot(G_0,'axis',[0 2 0 2 0 2],'frame','0', 'color', 'k', 'length', 0.4);
    % hold on;
    % trplot(T,'color', 'b', 'frame', 'D', 'length', 0.4);
    % trplot(T*G_DH,'color', 'r', 'frame', 'H', 'length', 0.4);
    % trplot(T*G_DH_last, 'frame', 'H', 'color', 'g', 'length', 0.4);
    % title(sprintf('t = %.2f sec', t));
    % drawnow;
    % hold off;

    % % CREATE GIF
    % frame = getframe(gcf);
    % im = frame2im(frame);
    % [A, map] = rgb2ind(im, 256);
    % 
    % % Save to GIF
    % imwrite(A, map, 'animation.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);

end

% Homogeneous Transformation door to base frame at the end of the second move
G_0D_last = T;

%% Last Move (Third)

% Rotation Trajectory of knob using 5 degree polynomial
theta_lastmove = mytrajectory(thetaf_1, theta0_2,dt:dt:1.5); % Same as before for the dt
i = 1;

for t = t2+dt:dt:tf 

    % Calculate the Homogeneous Tranformation Knob relative to base frame for each dt
    T = G_0D_last * G_DH * trotx(theta_lastmove(i));    
    g_T{(t2/dt) + i +1} = T;
    i = i + 1 ;
    
    % % CREATE 3D plot with the position and rotation of each frame (SLOW)
    % trplot(G_0,'axis',[0 2 0 2 0 2],'frame','0', 'color', 'k', 'length', 0.4);
    % hold on;
    % trplot(G_0D_last,'color', 'b', 'frame', 'D', 'length', 0.4);
    % trplot(G_0D_last * G_DH,'color', 'r', 'frame', 'H', 'length', 0.4);
    % trplot(T, 'frame', 'H', 'color', 'g', 'length', 0.4);
    % title(sprintf('t = %.2f sec', t));
    % drawnow;
    % hold off;

    % % CREATE GIF
    % frame = getframe(gcf);
    % im = frame2im(frame);
    % [A, map] = rgb2ind(im, 256);
    % 
    % % Save to GIF
    % imwrite(A, map, 'animation.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);

end

%% Plot the trajectories of PART A

% Quaternion Components Knob relative to base frame
q0h_w = zeros(1, tf/dt + 1);
q0h_x = zeros(1, tf/dt + 1);
q0h_y = zeros(1, tf/dt + 1);
q0h_z = zeros(1, tf/dt + 1);

% Position Knob relative to base frame
p0h_x = zeros(1, tf/dt + 1);
p0h_y = zeros(1, tf/dt + 1);
p0h_z = zeros(1, tf/dt + 1);

for i = 1:(tf/dt + 1)
    T = g_T{i};           % Homogeneous transformation knob to base frame
    R = T(1:3, 1:3);      % Extract rotation matrix
    p0h_x(i) = T(1,4);    % Position in x axis
    p0h_y(i) = T(2,4);    % Position in y axis
    p0h_z(i) = T(3,4);    % Position in z axis

    % Convert to quaternion: [w x y z]
    quat_0h = rotm2quat(R);   
    q0h_w(i) = quat_0h(1);   % w
    q0h_x(i) = quat_0h(2);   % x
    q0h_y(i) = quat_0h(3);   % y
    q0h_z(i) = quat_0h(4);   % z
end

% Plot orientation Trajectory using unit quaternion
figure;
hold on;
plot(t0:dt:tf, q0h_w, 'k','LineWidth',1);
plot(t0:dt:tf, q0h_x, 'r','LineWidth',1);
plot(t0:dt:tf, q0h_y, 'g','LineWidth',1);
plot(t0:dt:tf, q0h_z, 'b','LineWidth',1);
xline(t1,'--k');
xline(t2,'--k');
xlim([-0.2 5.5])
ylim([-1 1])

legend('w', 'x', 'y', 'z');
xlabel('Time (sec)');
ylabel('Quaternion Component');
title('Orientation Trajectory');
grid on;

% Plot position trajectory
figure;
hold on;
plot(t0:dt:tf, p0h_x, 'r','LineWidth',1);
plot(t0:dt:tf, p0h_y, 'g','LineWidth',1);
plot(t0:dt:tf, p0h_z, 'b','LineWidth',1);
xline(t1,'--k');
xline(t2,'--k');
xlim([-0.2 5.5])
ylim([0.5 2.2])

legend('$P0h_{X}$', '$P0h_{Y}$', '$P0h_{Z}$', 'interpreter','latex');
xlabel('Time (sec)');
ylabel('Positions');
title('Position Trajectory');
grid on;

%return;
%% Part B

% Plot the knob frames in start and end of full move
figure;

trplot(G_0H,'axis',[0 2 0 2 0 2],'color', 'r', 'frame', 'H', 'length', 0.4);
hold on;
trplot(G_0D_last * G_DH,'color', 'r', 'frame', 'H', 'length', 0.4);

% Start q
q0 = [-1.7752 -1.1823 0.9674 0.2149 1.3664 1.5708];
q = zeros(tf/dt +1, 6);
q(1,:) = q0;  
q_dot = zeros(tf/dt, 6);

% Our robot
ur10 = ur10robot() ;

t = t0;
%view(45,30);
for i = 1:(tf/dt)
    
    % The Homogeneous Tranformation of the current and the next pose
    T1 = g_T{i} * G_HE;
    T2 = g_T{i+1} * G_HE;
    
    % Calculate the twist
    deltaT = tr2delta(T1, T2);
    v_e = deltaT / dt;

    % Jacob of end-effector
    J = ur10.jacobe(q(i,:));
    
    % Calculate the angular velocities of the robot's revolute joints 
    q_dot(i,:) = (pinv(J) * v_e)';
    
    % Calculate the next position of the joints
    q(i+1,:) = q(i,:) + q_dot(i,:) * dt;
    
    % ur10.plot(q(i,:),'workspace',[0 2 0 2.5 0 1.5]);
    % Uncomment this to get the time as title
    % title(sprintf('t = %.2f sec', t));

    % % CREATE GIF 
    % frame = getframe(gcf);
    % im = frame2im(frame);
    % [A, map] = rgb2ind(im, 256);
    % 
    % % Svae to GIF
    % if t == 0
    %     imwrite(A, map, 'animation_robot.gif', 'gif', 'LoopCount', Inf, 'DelayTime', 0.05);
    % else
    %     imwrite(A, map, 'animation_robot.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    % end

    t = t + dt;
end

% Plot the robot moving
% ur10.plot(q,'workspace',[0 2 0 2.5 0 1.5]);

%% Plots 

% Quaternion Components for end-effector relative to base frame
q0e_w = zeros(1, tf/dt + 1);
q0e_x = zeros(1, tf/dt + 1);
q0e_y = zeros(1, tf/dt + 1);
q0e_z = zeros(1, tf/dt + 1);

% Quaternions Components for end-effector relative to knob
qhe_w = zeros(1, tf/dt + 1);
qhe_x = zeros(1, tf/dt + 1);
qhe_y = zeros(1, tf/dt + 1);
qhe_z = zeros(1, tf/dt + 1);

% Position end-effector relative to base frame
p0e_x = zeros(1, tf/dt + 1);
p0e_y = zeros(1, tf/dt + 1);
p0e_z = zeros(1, tf/dt + 1);
% Position end-effector relative to knob 
phe_x = zeros(1, tf/dt + 1);
phe_y = zeros(1, tf/dt + 1);
phe_z = zeros(1, tf/dt + 1);


for i = 1:(tf/dt + 1)
    T = ur10.fkine(q(i,:));         % Homogeneous transformation end-effector relative to base frame
    T_he = inv(g_T{i}) * T.T;       % Homogeneous transformation end-effector relative to knob frame
    
    p0e_x(i) = T.t(1);    % Position in x axis
    p0e_y(i) = T.t(2);    % Position in y axis
    p0e_z(i) = T.t(3);    % Position in z axis
    
    phe_x(i) = T_he(1,4);
    phe_y(i) = T_he(2,4);
    phe_z(i) = T_he(3,4);

    % Convert to quaternion: [w x y z]
    quat_0e = rotm2quat(T.R);   
    q0e_w(i) = quat_0e(1);   % w
    q0e_x(i) = quat_0e(2);   % x
    q0e_y(i) = quat_0e(3);   % y
    q0e_z(i) = quat_0e(4);   % z
    
    quat_he = rotm2quat(T_he(1:3,1:3));   
    qhe_w(i) = quat_he(1);   % w
    qhe_x(i) = quat_he(2);   % x
    qhe_y(i) = quat_he(3);   % y
    qhe_z(i) = quat_he(4);   % z
    
end

% Plot orientation Trajectory using unit quaternion
figure;
hold on;
plot(t0:dt:tf, q0e_w, 'k','LineWidth',1);
plot(t0:dt:tf, q0e_x, 'r','LineWidth',1);
plot(t0:dt:tf, q0e_y, 'g','LineWidth',1);
plot(t0:dt:tf, q0e_z, 'b','LineWidth',1);
xline(t1,'--k');
xline(t2,'--k');
xlim([-0.2 5.5])
ylim([-1 1])

legend('w', 'x', 'y', 'z');
xlabel('Time (sec)');
ylabel('Quaternion Component');
title('Orientation Trajectory');
grid on;

% Plot position trajectory
figure;
hold on;
plot(t0:dt:tf, p0e_x, 'r','LineWidth',1);
plot(t0:dt:tf, p0e_y, 'g','LineWidth',1);
plot(t0:dt:tf, p0e_z, 'b','LineWidth',1);
xline(t1,'--k');
xline(t2,'--k');
xlim([-0.2 5.5])
ylim([0.5 2.2])

legend('$P0e_{X}$', '$P0e_{Y}$', '$P0e_{Z}$', 'interpreter','latex');
xlabel('Time (sec)');
ylabel('Positions');
title('Position Trajectory');
grid on;

% Plot orientation Trajectory using unit quaternion
figure;
hold on;
plot(t0:dt:tf, qhe_w, 'k','LineWidth',1);
plot(t0:dt:tf, qhe_x, 'r','LineWidth',1);
plot(t0:dt:tf, qhe_y, 'g','LineWidth',1);
plot(t0:dt:tf, qhe_z, 'b','LineWidth',1);
xline(t1,'--k');
xline(t2,'--k');
xlim([-0.2 5.5])
ylim([-1 1])

legend('w', 'x', 'y', 'z');
xlabel('Time (sec)');
ylabel('Quaternion Component');
title('Orientation Trajectory');
grid on;

% Plot position trajectory
figure;
hold on;
plot(t0:dt:tf, phe_x, 'r','LineWidth',1);
plot(t0:dt:tf, phe_y, 'g','LineWidth',1);
plot(t0:dt:tf, phe_z, 'b','LineWidth',1);
xline(t1,'--k');
xline(t2,'--k');
%xlim([-0.2 5.5])
%ylim([0.5 2.2])

legend('$Phe_{X}$', '$Phe_{Y}$', '$Phe_{Z}$', 'interpreter','latex');
xlabel('Time (sec)');
ylabel('Positions');
title('Position Trajectory');
grid on;

% Plot error of relative position
figure;
hold on;
plot(t0:dt:tf, phe_x-0.1, 'r','LineWidth',1);
plot(t0:dt:tf, phe_y-0.1, 'g','LineWidth',1);
plot(t0:dt:tf, phe_z, 'b','LineWidth',1);
xline(t1,'--k');
xline(t2,'--k');

legend('$e_{X}$', '$e_{Y}$', '$e_{Z}$', 'interpreter','latex');
title('Error of relative position');
xlabel('Time (sec)');
ylabel('Error');
grid on;


% Position and Velocity responses

figure;
plot(t0:dt:tf, q,'LineWidth',1);
xlabel('Time (sec)'); 
ylabel('Position (rad)');
title('Positions responses');
xline(t1,'--k');
xline(t2,'--k');
legend('q1','q2','q3','q4','q5','q6','Location','best');
grid on;

figure;
plot(t0:dt:tf-dt, q_dot,'LineWidth',1);
xlabel('Time (sec)');
ylabel('Velocity (rad/s)');
title('Velocity responses');
xline(t1,'--k');
xline(t2,'--k');
legend('$\dot{q}1$','$\dot{q}2$','$\dot{q}3$','$\dot{q}4$',...
    '$\dot{q}5$','$\dot{q}6$','Interpreter','latex');
grid on;