function draw_drone(x,y,z,phi,theta,psi)
global l
% Function for drawing the drone on 3d.
%
% Inputs
% l distance between the center and the drone.
    % Traslation.
    % x coordinate of the center.
    % y coordinate of the center.
    % z coordinate of the center.
    % Rotation
    % phi: roll angle.
    % theta: pitch angle.
    % psi: yaw angle.
% Outputs    
    % Drawing of the drone.
%% Transform the coordinate.
% define the eart frame coordinates.
p0 = [x;y;z];
p1b = [l;0;0];
p2b = [-l;0;0];
p3b = [0;+l;0];
p4b = [0;-l;0];
% To plot the coordinate axis.
p1ob = [l/4;0;0];
p2ob = [-l/4;0;0];
p3ob = [0;l/4;0];
p4ob = [0;-l/4;0];
p5ob = [0;0;l/8];
p6ob = [0;0;-l/8];

% Convert the points.
% Earth frame coordinates.
p1e = transform_coord(p1b,phi,theta,psi);
p2e = transform_coord(p2b,phi,theta,psi);
p3e = transform_coord(p3b,phi,theta,psi);
p4e = transform_coord(p4b,phi,theta,psi);
% Coordinate lines.
% To plot the coordinate axis.
p1oe = transform_coord(p1ob,phi,theta,psi);
p2oe = transform_coord(p2ob,phi,theta,psi);
p3oe = transform_coord(p3ob,phi,theta,psi);
p4oe = transform_coord(p4ob,phi,theta,psi);
p5oe = transform_coord(p5ob,phi,theta,psi);
p6oe = transform_coord(p6ob,phi,theta,psi);

%% Draw the lines.
% Plot the center.
plot3(x,y,z,'ob','LineWidth',7,'MarkerSize',7);
hold on; 
% Define the points.
% Earth frame coordinates.
A1 = p1e+p0;
A2 = p2e+p0; 
A3 = p3e+p0;
A4 = p4e+p0;
% coordinates axis.
Ao1 = p1oe+p0;
Ao2 = p2oe+p0; 
Ao3 = p3oe+p0;
Ao4 = p4oe+p0;
Ao5 = p5oe+p0;
Ao6 = p6oe+p0;

% x coordinate.
plot3([A1(1) A2(1)],[A1(2) A2(2)],[A1(3) A2(3)],'-ob','LineWidth',5,'MarkerSize',7);
plot3([Ao1(1) Ao2(1)],[Ao1(2) Ao2(2)],[Ao1(3) Ao2(3)],'-r','LineWidth',5,'MarkerSize',7);
% y coordinate.
plot3([A3(1) A4(1)],[A3(2) A4(2)],[A3(3) A4(3)],'-ob','LineWidth',5,'MarkerSize',7);
plot3([Ao3(1) Ao4(1)],[Ao3(2) Ao4(2)],[Ao3(3) Ao4(3)],'-g','LineWidth',5,'MarkerSize',7);
% z coordinate.
plot3([Ao5(1) Ao6(1)],[Ao5(2) Ao6(2)],[Ao5(3) Ao6(3)],'-k','LineWidth',5,'MarkerSize',7);

%% Draw the circles.
draw_circle(A1,'c',phi,theta,psi);        % Draw circle for point 1 
draw_circle(A2,'c',phi,theta,psi);        % Draw circle for point 2 
draw_circle(A3,'c',phi,theta,psi);        % Draw circle for point 3 
draw_circle(A4,'c',phi,theta,psi);        % Draw circle for point 4 

daspect([1 1 3]) %Relación de aspecto
axis([-1 1 -1 1 0 3]) % Visualization range.
grid on %Rejilla
grid minor
hold off;
