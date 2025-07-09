close all
clear
clc

%% MOTOR DATA 

% LOAD AMK MOTOR DATA TABLES
% these tables are graciously provided by AMK
motor_data = load("A2370DD_T80C.mat","Shaft_Torque", "Stator_Current_Line_RMS", "Power_Factor", "Voltage_Line_RMS", "Iq_RMS", "Speed");
rpm_matrix = motor_data.Speed;
stator_current_line_RMS_matrix = motor_data.Stator_Current_Line_RMS;
shaft_torque_matrix = motor_data.Shaft_Torque;
voltage_line_RMS_matrix = motor_data.Voltage_Line_RMS;
power_factor_matrix = motor_data.Power_Factor; 
Iq_RMS_matrix = motor_data.Iq_RMS; 

%%power consumption calculated using formula for 3 phase motor:
% p = root 3 x V x I x PF x efficiency 

power_consumption_matrix = sqrt(3) * stator_current_line_RMS_matrix .* voltage_line_RMS_matrix .* power_factor_matrix;

% CREATE ROW AND COLUMN VECTORS FOR MOTOR DATA TABLE
% all motor parameter tables are a function of rpm and current
% where the rows represent RPM vlaues from 0 to 20000 in increments of 100
% and the columns represent stator line current RMS from 0A to 105A in
% increments of 5.25A as given by AMK
RPM = 0:100:20000; % row 
current = 0:5.25:105; % columns
% Generate a grid of RPM and current values for the table
[current_grid, RPM_grid] = meshgrid(current, RPM);

figure(1); 

mesh(current, RPM, shaft_torque_matrix); 
title("Shaft Torque vs RPM vs Current"); 
xlabel("Stator-line-current (A)"); 
ylabel("RPM * 1000");  
zlabel("Motor Shaft Torque (N*M)");


figure(2); 
mesh(current, RPM, voltage_line_RMS_matrix); 
title("Voltage Line RMS vs RPM vs Current"); 
xlabel("Stator-line-current (A)"); 
ylabel("RPM * 1000");  
zlabel("Voltage line RMS (V)");


figure(3); 
mesh(current, RPM, power_consumption_matrix); 
title("Power Consumption vs RPM vs Current"); 
xlabel("Stator-line-current (A)"); 
ylabel("RPM * 1000");  
zlabel("Power Consumption (W)");

