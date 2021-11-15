function draw_drone_3d(x,y,z,l)
%Función para dibujar una aproximación de un arco con un polígono
%
%Entradas
% x coordinate of the center.
% y coordinate of the center.
% l distance between the center and the drone.

%% Draw the lines.
% Plot the center.
hold on; 
axis([-10 10 -10 10 0 2]);
plot3(x,y,z,'ob','LineWidth',7,'MarkerSize',7);
% x coordinate.
plot3([x+l x-l],[0 0],[z z],'--ob','LineWidth',5,'MarkerSize',7);
          % To plot more data. 
% y coordinate.
plot3([0 0],[y+l y-l],[z z],'--ob','LineWidth',5,'MarkerSize',7);
%% Draw the circles.
draw_circle(x+l,0,0,'c');        % Draw circle for point 1 
draw_circle(x-l,0,0,'c');        % Draw circle for point 2 
draw_circle(0,y+l,0,'c');        % Draw circle for point 3 
draw_circle(0,y-l,0,'c');          % Draw circle for point 4 
% draw circle center.

hold off;
