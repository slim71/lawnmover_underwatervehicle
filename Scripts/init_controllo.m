galleggiamento = 66.5386;
saturation_heave = 135;
areaOfInterestCorner = [42.317744;10.915136; 0];
turning_surge = 0.64; %condizione iniziale per l integratore surge per andare a 0.2 m/s lungo la curva
%% Positioning controller
% surge_controller 
K_p_surge_pos = 13.7782*2.5;
K_i_surge_pos = 0.1679*30;
K_p_x_pos = 0.12;
% sway_controller
K_p_sway_pos = 90;
K_i_sway_pos = 5;
K_p_y_pos = 0.34;

Kq_p_sway_pos = [1.5735 1.0000];
Kq_i_sway_pos = -0.1;

% heave_controller
K_p_heave_pos = 320;
K_i_heave_pos = 1;
K_p_z_pos = 0.2;

Kq_p_heave_pos = [1.0592 1.0000];
Kq_i_heave_pos = -0.1;
% roll_controller
K_p_roll_pos = 0.1*40;
K_i_roll_pos = 0.1*0;
K_p_phi_pos = 1*0.1;

Kq_p_roll_pos = [2.7599 2.1213];
% pitch_controller
K_p_pitch_pos = 25;
K_i_pitch_pos = 0.1*0;
K_p_th_pos = 0.1;

Kq_p_pitch_pos = [0.3 0.01];
% yaw_controller
K_p_yaw_pos = 20;
K_i_yaw_pos = 1;
K_p_psi_pos = 0.25;

Kq_p_yaw_pos = [5.1787 1.0050]*0.5;
%% Diving controller
% surge_controller
K_p_surge_div = 80;
K_i_surge_div = 3; 
K_p_x_div = 0.12;

Kq_p_surge_div = [9.1789 1.0000]*0.5;
Kq_i_surge_div = -0.04;
% sway_controller
K_p_sway_div = 100;
K_i_sway_div = 6;
K_p_y_div = 0.6;

Kq_p_sway_div = [1.5735 1.0000];
Kq_i_sway_div = -0.1;

% heave_controller
K_p_heave_div = 200;
K_i_heave_div = 20;
K_p_z_div = 0.35;

% roll_controller
K_p_roll_div = 10;
K_i_roll_div = 0;
K_p_phi_div = 0.15;

Kq_p_roll_div = [2.7599 2.1213];
% pitch_controller
K_p_pitch_div = 50;
K_i_pitch_div = 0;
K_p_th_div = 0.3;

Kq_p_pitch_div = [19.0216 7.0711];
% yaw_controller
K_p_yaw_div = 70;
K_i_yaw_div = 0;
K_p_psi_div = 0.3;

Kq_p_yaw_div = [5.1787 1.0050];
%% Transect controller
% surge_controller
K_p_surge_trans = 20;
K_i_surge_trans = 0.1679*15;
% sway_controller
K_p_sway_trans = 80;
K_i_sway_trans = 2;
K_p_y_trans = 0.3;
% K_p_sway_trans = 10;
% K_i_sway_trans = 1;
% K_p_y_trans = 1;



Kq_p_sway_trans = [1.5735 1.0000];
Kq_i_sway_trans = -0.1;

% heave_controller
K_p_heave_trans = 170;
K_i_heave_trans = 5;
K_p_z_trans = 0.10;

Kq_p_heave_trans = [1.0592 1.0000];
Kq_i_heave_trans = -0.1;
% roll_controller
K_p_roll_trans = 0.1*40;
K_i_roll_trans = 0;
K_p_phi_trans = 0.1;

Kq_p_roll_trans = [2.7599 2.1213];
% pitch_controller
K_p_pitch_trans = 25;
K_i_pitch_trans = 0;
K_p_th_trans = 0.1;

Kq_p_pitch_trans = [19.0216 7.0711];
% yaw_controller
K_p_yaw_trans = 160;
K_i_yaw_trans = 20;
K_p_psi_trans = 0.2;

Kq_p_yaw_trans = [5.1787 1.0050];
%% Turning controller
% surge_controller
K_p_surge_turn = 150;
K_i_surge_turn= 0.1679*60;
% sway_controller
% K_p_sway_turn = 210;
% K_p_sway_turn = 180;
% K_i_sway_turn = 10;
% K_p_y_turn = 0.4;
K_p_sway_turn = 200;
K_i_sway_turn = 50;
K_p_y_turn = 0.3;

Kq_p_sway_turn = [1.5735 1.0000];
Kq_i_sway_turn = -0.1;
% heave_controller
K_p_heave_turn = 170;
K_i_heave_turn = 5;
K_p_z_turn = 0.1;

Kq_p_heave_turn = [1.0592 1.0000];
Kq_i_heave_turn = -0.1;
% roll_controller
K_p_roll_turn = 4;
K_i_roll_turn = 0;
K_p_phi_turn = 0.1;

Kq_p_roll_turn = [2.7599 2.1213];
% pitch_controller
K_p_pitch_turn = 25;
K_i_pitch_turn = 0;
K_p_th_turn = 0.1;

Kq_p_pitch_turn = [19.0216 7.0711];
% yaw_controller
% K_p_yaw_turn = 50;
% K_i_yaw_turn = 1.5;
K_p_yaw_turn = 60;
K_i_yaw_turn = 3;
%% Surfacing controller
% surge_controller
K_p_surge_surf = 80;
K_i_surge_surf = 3;
K_p_x_surf = 0.12;

Kq_p_surge_surf = [9.1789 1.0000];
Kq_i_surge_surf = -0.04;
% sway_controller
K_p_sway_surf = 80;
K_i_sway_surf = 3;
K_p_y_surf = 0.1;

Kq_p_sway_surf = [1.5735 1.0000];
Kq_i_sway_surf = -0.1;
% heave_controller
K_p_heave_surf = 200;
K_i_heave_surf = 20;
K_p_z_surf = 0.35;
% roll_controller
K_p_roll_surf = 10;
K_i_roll_surf = 1;
K_p_phi_surf = 0.15;

Kq_p_roll_surf = [2.7599 2.1213];
% pitch_controller
K_p_pitch_surf = 50;
K_i_pitch_surf = 0;
K_p_th_surf = 0.3;

Kq_p_pitch_surf = [19.0216 7.0711];
% yaw_controller
K_p_yaw_surf = 70;
K_i_yaw_surf = 0;
K_p_psi_surf = 0.3;

Kq_p_yaw_surf = [5.1787 1.0050];


















