%% Robotics Lab course
% Lab 2 - Pendubot Control
% Team 4
%       Name: Eirini-Maria Georganta, A.M.: 02121201
%       Name: Georgios Kassavetakis,  A.M.: 02121203
%       Name: Georgios Krommydas,     A.M.: 02121208
%       Name: Frantzieska Michail,    A.M.: 02121216

clear;
clc;

%% Pendubot State Characteristics for Kp = 1

Kd = [0 0.15 0.23];
ess = [18.768 19.488 19.632];
trt = [0.18 0.24 0.48];
overshoot = [0.1156 0.0188 0.0062];

figure(1);
clf;
plot(Kd, ess, 'b-*');
grid on;
title("Error State for K_P = 1");
xlabel("K_D");
ylabel("e_s_s (deg)");


figure(2);
clf;
plot(Kd, trt, 'b-*');
grid on;
title("Rise Time for K_P = 1");
xlabel("K_D");
ylabel("t_r_t (sec)");

figure(3);
clf;
plot(Kd, overshoot, 'b-*');
grid on;
title("Overshoot for K_P = 1");
xlabel("K_D");
ylabel("Overshoot (rad)");


%% Critical Region Analysis for Kd

Kp = [1 5 10 20]';
Kd_critical = [0.23 0.48 0.7 1.3]';
b = regress(Kd_critical,[ones(size(Kp)),Kp]);
Kd_critical_desired = [ones(size(Kp)),Kp]*b;
% Kd_critical_desired = a*sqrt(Kp)+b;
figure(4);
clf;
plot(Kp, Kd_critical, 'b-*');
hold on;
plot(Kp, Kd_critical_desired, 'r-*');
grid on;
title("Gain Parameters for Critical Response of the System");
xlabel("K_P");
ylabel("K_D");
legend("K_D Critical Values", "K_D Critical Desired Values","Location","northwest");