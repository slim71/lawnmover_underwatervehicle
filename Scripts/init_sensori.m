%% Mission A: Lawn-mower survey
disp('Loading sensors and environment parameters...')

%% Area di interesse
%% Condizioni iniziali (origine Ned_frame)
lat0 =  42.317744; 
lon0 = 10.915136; 
h0=0; 
NED_origin = [lat0; lon0; h0];
%dimensioni semiassi ellissoide 
a = 1.10; %[m]
b = 0.15; %[m]
c = 0.15; %[m]



%% Caratteristiche sonar
beam_width = deg2rad(15); % [rad]

% fondoscala sonar
range = 300; % [mt]

tilt_down = deg2rad(0); %[rad] angolo FLS_down 
tilt_front = deg2rad(45); %[rad] angolo di inclinazione FLS_front 


%% DEFINIZIONE FONDALE E VARIABILI GLOBALI 
global xfond4 yfond4 zfond4 tfond4 fondale 
syms xfond4 yfond4 zfond4 tfond4 real

mu = [100, 75];
sigma = 50*[1,0; 0,3];
fondale = 2500*exp(-0.5*([xfond4, 150 - yfond4] - mu)*inv(sigma)*([xfond4, 150 - yfond4] - mu)')/(2*pi*sqrt(det(sigma)));
mu = [160, 130];
sigma = 50*[1,0;0,2];
fondale = fondale + 2500*exp(-0.5*([xfond4, 150 - yfond4] - mu)*inv(sigma)*([xfond4, 150 - yfond4] - mu)')/(2*pi*sqrt(det(sigma)));
mu = [90, 30];
sigma = 140*[10,0;0,2];
fondale = fondale - 30000*exp(-0.5*([xfond4, 150 - yfond4] - mu)*inv(sigma)*([xfond4, 150 - yfond4] - mu)')/(2*pi*sqrt(det(sigma)));
mu = [50, 120];
sigma = 140*[10,0;0,2];
fondale = fondale + 30000*exp(-0.5*([xfond4, 150 - yfond4] - mu)*inv(sigma)*([xfond4, 150 - yfond4] - mu)')/(2*pi*sqrt(det(sigma)));
fondale = fondale + 2.5*cos((sin(xfond4/20)*sin(0.05*xfond4-0.05*(150 - yfond4))+5*cos(0.1*xfond4+0.2*(150 - yfond4)))/5) - 40;
fondale = - fondale;



%% Posizioni sensori 
% Imu,ahrs , profondimetro li consideriamo nel centro del sistema body
p_transponder = [100;75;33.7943]; %posizione transponder sul fondale (ned_frame)
p_gps = [0; 0; -c]; 
p_sonar_down = [0;0;c]; 
p_sonar_front = [a;0;0];
p_dvl = [0.2; 0 ; c];
p_usbl = [-0.2; 0 ; c];


%% Risoluzioni
res_gps = [0.01; 0.01; 0.01];   % [m]
res_profondimetro = 0.01;       % [m]
res_sonar_down = 0.006;         % [m]
res_sonar_front = 0.006;        % [m]
res_dvl = [0.001; 0.001; 0.001];  % [m/s]
res_ahrs = [deg2rad(0.01); deg2rad(0.01); deg2rad(0.01)];   % [rad]
res_usbl = [deg2rad(1); deg2rad(1); 0.1];       %[rad; rad; m] [B-E-R]
res_giroscopio = [deg2rad(0.02); deg2rad(0.02); deg2rad(0.02)];  % [rad/s]

%% Tempi di campionamento 
Ts_gps = 1;
Ts_profondimetro = 0.05;     
Ts_sonar_down = 0.5;
Ts_sonar_front = 0.5;
Ts_dvl = 1/7;                   
Ts_ahrs = 0.05;                
Ts_giroscopio = 0.05;          
Ts_usbl = 2;                    

%% Precisioni
var_dvl = [(0.005^2); (0.005^2); (0.005^2)];     %vx,vy,vz
var_profondimetro = (0.05)^2;
var_ahrs = [(deg2rad(0.1))^2; (deg2rad(0.1))^2; (deg2rad(1))^2]; %roll,pitch,yaw
var_sonar_front = (0.15)^2; 
var_sonar_down = (0.15)^2;  
var_giroscopio  = [(deg2rad(0.16))^2;(deg2rad(0.16))^2;(deg2rad(0.16))^2];          % wx,wy,wz
var_usbl = [(deg2rad(3)^2); (deg2rad(3))^2; (0.1)^2]; % bearing,elevaton,range 
var_gps = [1;1;1];