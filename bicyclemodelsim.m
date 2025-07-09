%Experimental lapsim 
close all
clear
clc
%dist step
dp = 0.01;  %meters 

%gr sweep for selecting gear ratio 
gr_low = 8;
gr_high = 12;
gr_step = 0.5;
gr_sweep = gr_low:gr_step:gr_high;
Acceltime = zeros(size(gr_sweep,1)); 

%sweep for selecting power split
rear_bias_low = 0.20;
rear_bias_high = 1.0; 
rear_bias_step = 0.05;
rb_sweep = rear_bias_low:rear_bias_step:rear_bias_high;

%Accel length 
dist = 75; %m

%initialize acceleration, velocity and position vectors 

P = 0:dp:dist; 
T = zeros(1, size(P,2)); %seconds 
A = zeros(1,size(P,2)); 
V = zeros(1,size(P,2)); 

AccelG = zeros(1, size(P,2)); 

%Initialize RPM, grip, tire force and weight transfer vectors 

rpm = zeros(size(P,2),1); 
frontrpm = zeros(size(P,2),1); 
Force = zeros(1, size(P,2)); 
forcerear = zeros(1, size(P,2)); 
Fgriprear = zeros(1, size(P,2)); 
friction_torque_rear = zeros(1, size(P,2)); 
forcefront = zeros(1, size(P,2));  
Fgripfront = zeros(1, size(P,2)); 
friction_torque_front = zeros(1, size(P,2)); 
WTlong = zeros(1, size(P,2));  

%Initialize downforce, drag and rolling resistance 

downforce = zeros(1, size(P,2)); 
drag = zeros(1, size(P,2)); 
rollr = zeros(1,size(P,2)); 

%% AMK Motor Data and Calculations 

%this section is taken from Andrew Tschaens straight line sim. Dont fully
%understand it, but working towards it with help of Henry Greenwood.
%currently only confirmed working for RWD, since I'm not sure how the power
%will be spread among the motors, and how it effects the power consumption.

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
power_consumption_matrix = sqrt(3) * stator_current_line_RMS_matrix .* voltage_line_RMS_matrix .* power_factor_matrix;


%CREATE ROW AND COLUMN VECTORS FOR MOTOR DATA TABLE
%all motor parameter tables are a function of rpm and current
%where the rows represent RPM vlaues from 0 to 20000 in increments of 100
%and the columns represent stator line current RMS from 0A to 105A in
%increments of 5.25A as given by AMK
RPM = 0:100:20000; % row 
current = 0:5.25:105; % columns
% Generate a grid of RPM and current values for the table
[current_grid, RPM_grid] = meshgrid(current, RPM);

% INCREASE MOTOR TABLE RESOLUTION USING INTERPOLATION 
% we want finely stepped data for the simulation, the data provided is too
% coarse
RPM_fine = 0:1:20000; % Data for every 1 rpm
current_fine = 0:0.1:105; % Data for every 0.1 A
[current_fine_grid, RPM_fine_grid] = meshgrid(current_fine, RPM_fine);

% Calculate interpolated torque values for finely stepped RPM and current 
interpolatedTorque = interp2(current_grid, RPM_grid, shaft_torque_matrix, current_fine_grid, RPM_fine_grid, 'spline');
interpolatedVoltage = interp2(current_grid, RPM_grid, voltage_line_RMS_matrix, current_fine_grid, RPM_fine_grid, 'spline');
interpolatedCurrent = interp2(current_grid, RPM_grid, stator_current_line_RMS_matrix, current_fine_grid, RPM_fine_grid, 'spline');
interpolatedPowerConsumption = interp2(current_grid, RPM_grid, power_consumption_matrix, current_fine_grid, RPM_fine_grid, 'spline');
interpolatedIqRMS = interp2(current_grid, RPM_grid, Iq_RMS_matrix, current_fine_grid, RPM_fine_grid, 'spline');
interpolatedPowerFactor = interp2(current_grid, RPM_grid, power_factor_matrix, current_fine_grid, RPM_fine_grid, 'spline');
% the motor data is now more finely stepped

%initialize motor related data 

rear_motor_torque = zeros(size(P,2),1);
%front_motor_torque = zeros(size(P,2),1); 
rear_stator_current_line_RMS = zeros(size(P,2),1);
%front_stator_current_line_RMS = zeros(size(P,2), 1);
voltage_line_RMS = zeros(size(P,2),1);
power_loss = zeros(size(P,2),1);
motor_power_consumption = zeros(size(P,2),1);
I_qs = zeros(size(P,2), 1);
I_ds = zeros(size(P,2), 1);
battery_current = zeros(size(P,2), 1);
battery_capacity = zeros(size(P,2), 1);
SOC = zeros(size(P,2), 1);
inverter_DCV = zeros(size(P,2), 1);

%% Car Parameters  
%car parameters were taken from Andrews straight line sim, but I updated
%many of them based on infomration from the design brief as there were some
%noticable differences, namely the COG height, changing from 0.485 -> 0.295
 

%CONSTANTS
rho = 1.225; % kg/m^3
g = 9.81; % m/s^2

%MASS given RWD configuration 
mass = 268.8; % total mass [kg] (includes driver weight)

%AERODYNAMICS
cd = 1.28; % coefficient of drag
cdf = 3.75; % coefficient of downforce
frontal_A = 1.21; % frontal area [m^2]
Cp = 1-0.514; % center of pressure? not sure if this is necessary or a real thing. currently just an estimate given 
%the rear wing is very large. 
%VEHICLE DYNAMICS
rear_weight_distribution = 0.57; % Percent mass in rear
wheelbase = 1.535; % wheel base [m]
COG_height = 0.295; %  COG height [m]
mux = 1.4; % average tire friction coefficient
tire_radius = 0.2; % effective tire radius [m]


% DRIVETRAIN DATA
num_motors = 2; % Number of motors being used
gearbox_e = 0.85; % Approx. Gearbox efficiency -> Multiply the  torque by this value
gearratio = 11.33;
frontgearratio = 11.33; 
max_power = 80;  %kW
torque = 21; %n*m
rear_bias = 1.0; % how much of the power is sent to the rear wheels 
max_rpm = 19200; %max rpm 


% INVERTER DATA
inverter_e = 0.98; % Approximated inverter efficiency

% BATTERY DATA   
s_count = 145; % num of battery cells in series
p_count = 3; % num of battery parallel rows
cell_DCIR = 0.010; % ohms
battery_resistance = s_count * cell_DCIR / p_count; % OHMS
cell_max_voltage = 4.1; % maximum voltage we charge each cell
battery_OCV = cell_max_voltage * s_count; % Battery open current voltage


% %% Gear ratio sweep 
% i = 1; 
% for frontgearratio = gr_low:gr_step:gr_high

% %% Power split sweep 
% i = 1;  
% for rear_bias = rear_bias_low:rear_bias_step:rear_bias_high

% Initial AVP
A(1) = 0; 
V(1) = 0; 
P(1) = 0;

if rear_bias < 1.0 
%MASS given 4WD configuration (based on weight of drivetrain system found on design report) 
mass = 268.8 + 20;  

else 
mass = 268.8; 
end

%% QSS Timestep Calculations 

%This section calculates the forces on the car based on a "bicycle model",
%where force can be transferred longitudinally between the two wheels/axles
%of the car. 

%basically I am using a distance vector, and at each distance dp, i am
%updating the accel, velocity, position, time, and all of the other forces,
%etc at each step 


 
for pos = 1:1:size(P,2)-1
    % radians/s of tire based on velocity 
    %rps = V(pos)/tire_radius; 
    % motor rpm conversion 
    rpm(pos) = V(pos) * (60 /(2 * tire_radius * 3.141)) * gearratio; 
    frontrpm(pos) = V(pos) * (60 /(2 * tire_radius * 3.141)) * frontgearratio; 
    %longitudinal weight transfer 
    %comment: how does this work in initial condition, and also this shows
    %why it is called quasi steady state, because the weight transfer is
    %based on the AVP in the previous timestep, not the current one, so its
    %behind perhaps 
    

    %calculate and update downforce, drag and rolling resistance acting on the car 
    

    %downforce pushing car down and increasing grip 
    downforce(pos) = ((1/2) * cdf * V(pos)^2 * frontal_A * rho); 
    
    %drag counteracting car forward movement 
    drag(pos) = ((1/2) * cd * V(pos)^2 * frontal_A * rho); 

    %rolling resistance 
    rollr(pos) = 0.02 * (mass * g + downforce(pos)); 

    %weight transfer occuring as a result of acceleration 
    WTlong(pos) = 0.5* mass * A(pos) * (COG_height / wheelbase); %weight transfer 
    
    %rear grip considering load transfer during acceleration 
    Fgriprear(pos) = mux * ((mass * g * rear_weight_distribution)+(downforce(pos) * Cp) + WTlong(pos));
    %front grip considering load transfer during acceleration 
    Fgripfront(pos) = mux * ((mass * g * (1-rear_weight_distribution))+(downforce(pos) * (1-Cp)) - WTlong(pos));



    %available motor torque for rear motor from grip on ground (both tires
    %combined) 
    friction_torque_rear(pos) = Fgriprear(pos) * tire_radius / gearratio; 

    %available torque for front motors from grip on ground (both tires combined) 
    friction_torque_front(pos) = Fgripfront(pos) * tire_radius / gearratio; 
    

    %constant torque region until motor reaches max power (rpm), then
    %constant power region. the point at which the motor changes is known
    %as the base speed 
    %forcerear(pos) = min(gearbox_e * num_motors*torque*gearratio/tire_radius, gearbox_e*(max_power * rear_bias)* 1000 / V(pos));
   

    %plug values into motor simulation function. function outputs torque
    %for front and rear motors, power usage, current, voltage etc 

    [motor_power_consumption(pos), rear_motor_torque(pos), voltage_line_RMS(pos), rear_stator_current_line_RMS(pos), I_qs(pos), I_ds(pos), battery_current(pos), inverter_DCV(pos)] = testPwtFunction(rpm(pos), friction_torque_rear(pos), inverter_e, interpolatedTorque, interpolatedVoltage, interpolatedCurrent, interpolatedPowerConsumption, interpolatedPowerFactor, interpolatedIqRMS, battery_OCV, battery_resistance);


    forcerear(pos) = rear_motor_torque(pos) * gearratio * num_motors * gearbox_e / tire_radius; 

   % forcefront(pos) = front_motor_torque(pos) * frontgearratio * num_motors * gearbox_e / tire_radius; 
    
    %removes extra data point that makes graph look weird (comes from
    %dividing by zero) 

    if(rear_bias == 1.0)
    forcefront(1) = 0; 
    end 

    
    %fprintf("Available Grip: Rear - %0.2f Front - %0.2f \nAvailable Force: Rear - %0.2f Front - %0.2f \nWeight Transfer: %0.2f\n", Fgriprear(time), Fgripfront(time), forcerear(time), forcefront(time),WTlong);
   
  

    %calculated the total force pushing the car forward minus the forces counteracting the car  

    Force(pos) = min(forcerear(pos), Fgriprear(pos)) + min(forcefront(pos), Fgripfront(pos)) - drag(pos) - rollr(pos);  
 
    %Update kinematics
    
    A(pos+1) = Force(pos)/mass; 
    %rev limiter 
    if rpm(pos) > max_rpm | frontrpm(pos) > max_rpm
    
    A(pos+1) = 0; %car cannot accelerate further once max rpm is reached 

    end
    
    AccelG(pos+1) = A(pos+1)/g; 

    %update velocity based on acceleration 
    V(pos+1) = sqrt(V(pos)^2 + 2 * A(pos+1) * dp); 
    
   
    
    P(pos+1) = P(pos) + dp; %update postion 
  
    dt = dp/V(pos+1);     %Calculate how much time passes for each position interval  
    if V(pos+1) == 0 
        dt = 0; 
    end
    %updates time vector 
    T(pos+1) = T(pos) + dt; 
    
    
   

end 

% section taken from andrews sim. will understand better once I convert it
% so it works for 4WD. 


%% MOTOR EFFICIENCY AND HEAT GENERATION
mechanical_output_power = rear_motor_torque .* ((2 * pi * rpm / 60) / 1000); %kW
motor_efficiency = mechanical_output_power ./ motor_power_consumption;
motor_heat_generation = motor_power_consumption - mechanical_output_power; %kW
        
  
%% INVERTER ENERGY CONSUMPTION, HEAT GENERATION, TEMPERATURE
inverter_power_consumption = motor_power_consumption/inverter_e;
inverter_pc = inverter_DCV .* battery_current / 2;
      

        
%% BATTERY ENERGY CONSUMPTION, HEAT GENERATION, TEMPERATURE
battery_power_consumption = battery_OCV * battery_current;
battery_heat_generation = battery_current.^2 .* battery_resistance;
battery_energy_consumption = inverter_power_consumption .* dt; % J
battery_energy_consumption_kWh = battery_energy_consumption / 3.6e6; % kWh
total_battery_energy_consumption_kWh = sum(battery_energy_consumption_kWh);
battery_power_consumption = battery_power_consumption / 1000;


% % %Display 75 m acceleration time 
% fprintf("75 m acceleration event time: %0.2f for front gear ratio: %0.1f\n", T(pos+1), frontgearratio); 
% % fprintf("75 m acceleration event time: %0.2f for power split: %0.2f\n", T(pos+1), rear_bias); 
% Acceltime(i) = T(pos+1); 
% i=i+1; %counter variable 
% end
% 
% % Display figure for front gear ratio sweep
% 
% figure(1); 
% plot(gr_sweep, Acceltime, 'bo'); 
% title("75m acceleration event time for front gear ratio 10-18 assuming 50 50 power split"); 
% xlabel("Front Gear Ratio");
% ylabel("Time (s)"); 
% grid on; 
% 
% %% Display figure for front-rear power split sweep 
% 
% figure(1); 
% plot(rb_sweep, Acceltime, 'bo'); 
% title("75m acceleration event time for front-rear power split"); 
% xlabel("Power split (fraction of power to rear motors)");
% ylabel("Time (s)"); 
% grid on; 

%% Display single accel sim forces and traction graphs 

%plot AVP Graphs 
fprintf("75 m acceleration event time: %0.2f", T(pos+1)); 
figure(1); 

plot(T,AccelG, 'ro'); 

title("Acceleration vs Time"); 
xlabel("Time (seconds)"); 
ylabel("Acceleration (gs)"); 
grid on

figure(2); 

plot(T, V, 'g-'); 

title("Velocity vs Time"); 
xlabel("Time (seconds)"); 
ylabel("Velocity (m/s)"); 
grid on 
figure(3); 

plot(T, P, 'b-'); 

title("Position vs Time"); 
xlabel("Time (seconds)"); 
ylabel("Position (m)"); 
grid on
figure(4); 
hold on
plot(T, forcerear, 'b-'); 
plot(T, Fgriprear, 'r-'); 
title("Force & Grip from rear wheels vs Time"); 
xlabel("Time (seconds)"); 
ylabel("Force (N)"); 
legend("Available Power", "Available Traction"); 
grid on
hold off
figure(5); 
hold on
plot(T, forcefront, 'b-'); 
plot(T, Fgripfront, 'r-'); 
title("Force & Grip from front wheels vs Time"); 
xlabel("Time (seconds)"); 
ylabel("Force (N)"); 
legend("Available Power", "Available Traction"); 
grid on
hold off 


%% display motor characteristic graphs 
 total_motor_power_consumption = 2 * motor_power_consumption / 1000;
 total_mechanical_output_power = 2 * mechanical_output_power;


%% Motor Characteristics
figure(6)
subplot(4, 1, 1)
scatter(rpm, rear_motor_torque)
title("Acceleration Run [75m]")
xlabel("Motor RPM")
ylabel("Motor Torque [Nm]")
subtitle("Torque/RPM Data")
grid on

subplot(4, 1, 2)
scatter(rpm, rear_stator_current_line_RMS,"red")
xlabel("Motor RPM")
ylabel("Current RMS [A]")
hold on
scatter(rpm, I_qs, "blue")

scatter(rpm, I_ds, "yellow"); 

legend("Stator Current", "Torque-Producing Stator Current, Iq", "field weakening current, Id");
grid on
hold off 

subplot(4, 1, 3); 
hold on
plot(T, battery_power_consumption, 'ro'); 
plot(T, total_motor_power_consumption, 'bo'); 
plot(T, battery_heat_generation/1000, 'go');
legend("Battery Power Consumption", "Motor Power Consumption", "Battery Heat losses"); 
xlabel("time (sec)"); 
ylabel("power consumption (kw)"); 
grid on 

subplot(4, 1, 4)
scatter(rpm, motor_power_consumption)
grid on
xlabel("Motor RPM")
ylabel("Power [kW]")
title("Power vs RPM")
hold on
scatter(rpm, mechanical_output_power*1000)
legend("Electrical Input", "Mechanical Output")
grid on
hold off

% 
% 

%
% subplot(4, 1, 4)
% scatter(rpm, motor_power_consumption)
% grid on
% xlabel("Motor RPM")
% ylabel("Power [kW]")
% title("Power vs RPM")
% hold on
% scatter(rpm, mechanical_output_power)
% legend("Electrical Input", "Mechanical Output")
% grid on
% hold off
% 
% subplot(4, 1, 3)
% scatter(rpm, voltage_line_RMS)
% xlabel("Motor RPM")
% ylabel("Voltage Line RMS [V]")
% grid on
% 


% figure(7)
% scatter(rpm, battery_power_consumption)
% hold on
% xlabel("Motor RPM")
% ylabel("Battery Power Consumption [kW]")
% scatter(rpm, total_motor_power_consumption)
% hold on
% xlabel("Motor RPM")
% ylabel("Power [kW]")
% title("Power vs RPM")
% hold on
% scatter(rpm, total_mechanical_output_power)
% legend("Battery Power Consumption", "Electrical Input", "Mechanical Output")
% grid on
% hold off

