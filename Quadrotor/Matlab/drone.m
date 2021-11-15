clc;
clear;
close all;
warning('off','all')
warning
%% Drone Parameters.
global m l Ixx Iyy Izz Jr b d g k Imax Ts
m=0.494; % Kg
l=0.135;  % m
Ixx=2.458E-4;  
Iyy=4.090E-4;  
Izz=5.2256E-4;
Jr=5.225E-5;
b=7.66E-5;  % Thrust coefficient
d=5.63E-6;  % Drag coefficient
g=9.81;      % Gravety m/s*2
k=[100 0.01 100 10 120 45 0.03 111 0.1 20 40 25];
Imax=1000;
Ts = 0.1;  % Sample time
%% Controller gains.
kp = [30;25;20;20;20;15];
kd = [20;30;35;2;2;2];

%% Simulate simulink program
simopt = simset('solver','ode45','SrcWorkspace','Current','DstWorkspace','Current');  % Initialize sim options
sim('final_model',[0 250],simopt);
%% Plot data
%% Position 
% X Position and velocity.
figure(1)
subplot(321)
%plot(time.Time,references(:,1),time.Time,X(1,:),time.Time,dX(1,:));
plot(time.Time,references(:,1),time.Time,X(1,:));
title (' X ');
legend('Ref_x','X','location','southeast');
grid on 
grid minor
xlim([0 100])
ylim([-0.55 0.55])
ylabel('x [m]')
xlabel('t [s]')

% Y .
subplot(323)
%plot(time.Time,references(:,2),time.Time,Y(1,:),time.Time,dY(1,:));
plot(time.Time,references(:,2),time.Time,Y(1,:));
title (' Y ');
legend('Ref_x','Y','location','southeast');
grid on 
grid minor
xlim([0 100])
ylim([-0.55 0.55])
ylabel('y [m]')
xlabel('t [s]')

% Z
subplot(325)
%plot(time.Time,references(:,3),time.Time,Z(1,:),time.Time,dZ(1,:));
plot(time.Time,references(:,3),time.Time,Z(1,:));
title (' Z ');
legend('Ref_z','Z','location','southeast');
grid on 
grid minor
xlim([0 100])
ylim([-0.1 1.1])
ylabel('x [m]')
xlabel('t [s]')

% Roll
subplot(322)
plot(time.Time,references(:,4),time.Time,Phi(1,:));
title (' \phi ');
legend('Ref_{\phi}','\phi','location','southeast');
grid on 
grid minor
xlim([0 100])
ylim([-100 55])
ylabel('\phi [º]')
xlabel('t [s]')

% Pitch
subplot(324)
plot(time.Time,references(:,5),time.Time,Theta(1,:));
title (' \theta ');
legend('Ref_{\theta}','\theta','location','southeast');
grid on 
grid minor
xlim([0 100])
ylim([-100 100])
ylabel('\theta [º]')
xlabel('t [s]')

% Yaw
subplot(326)
plot(time.Time,references(:,6),time.Time,Psi(1,:));
title (' \psi ');
legend('Ref_{\psi}','\psi','location','southeast');
grid on 
grid minor
xlim([0 100])
ylim([-0.2 2.2])
ylabel('\psi [º]')
xlabel('t [s]')

%% 3D plot.
figure(7)
axis([-10 10 -10 10 0 2])

plot3([references(:,1) X(1,:)'],[references(:,2) Y(1,:)'],...
      [references(:,3) Z(1,:)'], 'LineWidth',2);
title (" Quadrotor trajectory ");
legend('Ref','Calculated');
grid on 
grid minor
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

%% animation.
close all;
n = length(X);
v = 0.0000001;          % Animation velocity.
% Define the name and the rate of the video
writerObj = VideoWriter('myVideo.avi');
writerObj.FrameRate = 20;
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(X)
    figure(8);
    draw_drone(X(1,i),Y(1,i),Z(1,i),Phi(1,i),Theta(1,i),Psi(1,i));
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    F(i) = getframe(gcf);
    writeVideo(writerObj, F(i));
    pause(v*(time.Time(3)-time.Time(2)));
end
% Finish the video.
close(writerObj);

%% Print the model
% final_model
% print -s -dsvg final_model_01
