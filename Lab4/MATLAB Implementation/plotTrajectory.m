function [] = plotTrajectory(tspan,pd,vd,ad,Name,figsave)
%plotTrajectory shows the asked plot on a new figure with the given name.
%   plotTrajectory(tspan,pd,vd,ad,name) shows the trajectory (position, 
%   velocity, acceleration) on the cartesian space.
%
%   Name variable is the figure name used for the figure created by the 
%   function.
%
%----------------------------PROJECT VERSION-------------------------------
figure('Name',Name,'NumberTitle','off','WindowState','maximized');
clf
subplot(3,3,1)
plot(tspan,pd(1,:),'b-')
xlabel('Time [s]')
ylabel('Position x [mm]')
grid on
subplot(3,3,2)
plot(tspan,vd(1,:),'b-')
xlabel('Time [s]')
ylabel('Velocity x [mm/s]')
grid on
subplot(3,3,3)
plot(tspan,ad(1,:),'b-')
xlabel('Time [s]')
ylabel('Acceleration x [mm/s^2]')
grid on
subplot(3,3,4)
plot(tspan,pd(2,:),'b-')
xlabel('Time [s]')
ylabel('Position y [mm]')
grid on
subplot(3,3,5)
plot(tspan,vd(2,:),'b-')
xlabel('Time [s]')
ylabel('Velocity y [mm/s]')
grid on
subplot(3,3,6)
plot(tspan,ad(2,:),'b-')
xlabel('Time [s]')
ylabel('Acceleration y [mm/s^2]')
grid on
subplot(3,3,7)
plot(tspan,pd(3,:),'b-')
xlabel('Time [s]')
ylabel('Position z [mm]')
grid on
subplot(3,3,8)
plot(tspan,vd(3,:),'b-')
xlabel('Time [s]')
ylabel('Velocity z [mm/s]')
grid on
subplot(3,3,9)
plot(tspan,ad(3,:),'b-')
xlabel('Time [s]')
ylabel('Acceleration z [mm/s^2]')
grid on
sgtitle('Position/Velocity/Acceleration Trajectory') 
if figsave
    str='Trajectory.jpg';
    saveas(gcf,str)
end

end