clear all
close all
clc

disp('Loading System parameters...')

%% LOAD Mission File
% file 'mission.m' is executed to load mission parameters (uncomment the right one)
run missionA.m;
% run missionB.m;
% run missionC.m;


%% Environmental data

rho = 1030;                 % [kg/m^3] Sea Water Density

seaCurrent = [0.1;0.05;0]; %[m/s] Sea current speed, values can be modified       

%% Data for thrusters [Inspired to Bluerobotics model T200]

D = 0.076;                  % [m] Propeller Diameter
n_max = 340;                % [rad/s] Maximum propeller rotational speed
dead_zone_limit = 31.5;     % [rad/s] Corresponding to about 350RPM
omega = 0.1;                % []  Wake Fraction Number

% kT(J0) function characterisation
alpha1 =  0.0113;          % [Ns^2/m/kg/rad^2]
alpha2 = -0.0091;          % [Ns^2/m/kg/rad]

a1 =  rho * D^4 * alpha1;
a2 = -rho * D^3 * alpha2 * (1 - omega);

%% Mission Supervisor & Reference Generator parameters
% here, the Mission Supervisor & Reference Generator parameters parameters are included

run Supervisor.m

%% Vehicle Model parameters
% here, the Vehicle Model parameters are included
run G3_data_current_def.m

%% Environment Model & Sensor Model parameters
% here, the Environment Model & Sensor Model parameters parameters are included
run init_sensors.m

%% Controller parameters
% here, the Controller parameters are included
run controller_1.m%parametri_controllori.m

%% Navigation System parameters
% here, the Navigation System parameters are included
run init_navigation.m

%% System parameters loaded
disp('Done')