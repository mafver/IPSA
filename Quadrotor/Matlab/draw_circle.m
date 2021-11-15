function draw_circle(pc,color,phi,theta,psi)
%Función para dibujar una aproximación de un arco con un polígono
%
%Entradas
% pc: 3d coordinates of the center.
% l distance between the center and the drone.
%% Parameters.
R = 0.06;      % Circle radio
%% Define the vectors.
va = 0:0.01:2*pi;
vx = zeros(length(va),1);
vy = zeros(length(va),1);
vz = zeros(length(va),1);
for i=1:length(va)
    vb(1,1) = R.*cos(va(i));
    vb(2,1) = R.*sin(va(i));
    vb(3,1) = 0;
    % Change the coordinate frame.
    ve = transform_coord(vb,phi,theta,psi);
    vx(i) = pc(1) + ve(1);
    vy(i) = pc(2) + ve(2);
    vz(i) = pc(3) + ve(3);
end
%% Plot the circle.
patch(vx,vy,vz,color);
hold on
%% Plot the center.
plot3(pc(1),pc(2),pc(3),'ob','LineWidth',4,'MarkerSize',3); % center point.

