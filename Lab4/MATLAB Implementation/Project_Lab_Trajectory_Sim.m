%% Robotics ROS Project
clc
clear
close all

%% Parameter Init

%Basic length and D-H parameters for the Robot Manipulator
l=[26.7,29.3,5.25,35.12,12.32]; %cm
theta_1 = 0.2225;
theta_2 = 0.6646;
dhParameters=[];
dhParameters.d = [l(1);0;l(2);0;l(4)*cos(theta_1);0;l(5)*cos(theta_2)];
dhParameters.theta = [0;0;0;0;0;0;0];
dhParameters.a = [0;0;0;l(3);l(4)*sin(theta_1);0;l(5)*sin(theta_2)];
dhParameters.alpha = [0;-pi/2; pi/2;pi/2;pi/2; pi/2;-pi/2];

% Time vector creation
dt = 0.001; %i.e. 1 msec sampling period
T = 20; %Movement Period
Tf=T/2; 	%Linear Movement duration in seconds
tspan_1=0:dt:Tf; %Time of the 1st part of motion
tspan_2=Tf:dt:(2*Tf); %Time of the 2nd part of motion

p0 = [60.43,20,15.08]';
pf = [60.43,-20,15.08]';
Parameters = [];
Parameters.StartPosition=p0;
Parameters.StartVelocity=[0,0,0]';
Parameters.StartAcceleration=[0,0,0]';
Parameters.EndPosition=pf;
Parameters.EndVelocity=[0,0,0]';
Parameters.EndAcceleration=[0,0,0]';
Parameters.Parameterization='parabMix';
Parameters.PhaseTime=1;
[pd_1,vd_1,ad_1] = trajGeneration(Parameters,tspan_1,'Cartesian','Line');
Parameters.StartPosition=pf;
Parameters.EndPosition=p0;
[pd_2,vd_2,ad_2] = trajGeneration(Parameters,tspan_2,'Cartesian','Line');
pd=[pd_1(:,1:(end-1)),pd_2];
vd=[vd_1(:,1:(end-1)),vd_2];
ad=[ad_1(:,1:(end-1)),ad_2];
tspan=[tspan_1(1:(end-1)),tspan_2];
plotTrajectory(tspan,pd,vd,ad,'Trajectory On Cartesian Space',false);

