%% QSS BICYCLE STRAIGHT LINE SIMULATION W/ AWD 

%mostly adapted from Andrew's straight line sim, but modified so that it
%works with 4WD 

close all
clear
clc
%dist step
dp = 0.01;  %meters 

%gr sweep for selecting gear ratio 
gr_low = 8;
gr_high = 18;
gr_step = 0.5;
gr_sweep = gr_low:gr_step:gr_high;
Acceltime = zeros(size(gr_sweep,1));  

% for determining max power at each gear ratio 

maxpower = zeros(size(gr_sweep,1)); 

%difference between 80 kW and max power 

maxpowerdelta = zeros(size(gr_sweep,1)); 

%taken from max power delta lol 

%top speed 

topspeed = zeros(size(gr_sweep,1)); 

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

rear_motor_rpm = zeros(size(P,2),1); 
front_motor_rpm = zeros(size(P,2),1); 

Force = zeros(1, size(P,2)); 

% Rear torque and force parameters 

forcerear = zeros(1, size(P,2)); 
Fgriprear = zeros(1, size(P,2)); 
rear_friction_torque = zeros(size(P,2),1); 

% Front torque and force parameters 

forcefront = zeros(1, size(P,2));  
Fgripfront = zeros(1, size(P,2)); 
front_friction_torque = zeros(size(P,2),1); 

%Weight transfer vector 

WTlong = zeros(1, size(P,2));  

%Initialize downforce, drag and rolling resistance 

downforce = zeros(1, size(P,2)); 
drag = zeros(1, size(P,2)); 
rollr = zeros(1,size(P,2)); 

%% AMK Motor Data and Calculations 

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

%    Rear motors 

rear_motor_torque = zeros(size(P,2),1);
rear_stator_current_line_RMS = zeros(size(P,2),1);
rear_voltage_line_RMS = zeros(size(P,2),1);
rear_motor_power_consumption = zeros(size(P,2),1);
rear_I_qs = zeros(size(P,2), 1);
rear_I_ds = zeros(size(P,2), 1); 


%   Front Motors 

front_motor_torque = zeros(size(P,2),1); 
front_stator_current_line_RMS = zeros(size(P,2), 1);
front_voltage_line_RMS = zeros(size(P,2),1); 
front_motor_power_consumption = zeros(size(P,2),1); 
front_I_qs = zeros(size(P,2), 1);
front_I_ds = zeros(size(P,2), 1); 

%  Overall system 


battery_current = zeros(size(P,2), 1);
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
max_power = 80000; %W
%torque = 21; %n*m
rear_bias = 0; % how much of the power is sent to the rear wheels. for now 0 means AWD, 1.0 means RWD
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


%%%%%%%%%%%%%%%%%%%%%%%%

%% Gear ratio sweep 
i = 1; 
for gearratio = gr_low:gr_step:gr_high

frontgearratio = gearratio; 



%%%%%%%%%%%%%%%%%%%%%%%%%


% %% Power split sweep 
% i = 1;  
% for rear_bias = rear_bias_low:rear_bias_step:rear_bias_high

% Initial AVP
A(1) = 0; 
V(1) = 0; 
P(1) = 0;

if rear_bias < 1.0 
%MASS given 4WD configuration (based on weight of drivetrain system found on design report) 

%rears are 13 kg combined, front is 5 kg combined 

mass = 268.8 + 8;  

else 
mass = 268.8; 
end

%% QSS Timestep Calculations 

%This section calculates the forces on the car based on a "bicycle model",
%where force can be transferred longitudinally between the two wheels/axles
%of the car. 

%What is quasi-steady state? A qss model is one that assumes that given a
%small enough timestep, you can ignore transient effects like tires, and
%suspension, and assume that parameters are constant for the sake of
%calculations. 

%basically I am using a distance vector, and at each distance dp, i am
%updating the accel, velocity, position, time, and all of the other forces,
%etc at each step 

% are we derating our motors 
derate = 0; 
 
% are we hitting the power limit 

power_limit = 0; 

for pos = 1:1:size(P,2)-1
  
    % motor rpm conversion 
    rear_motor_rpm(pos) = V(pos) * (60 /(2 * tire_radius * 3.141)) * gearratio; 
    front_motor_rpm(pos) = V(pos) * (60 /(2 * tire_radius * 3.141)) * frontgearratio; 
 

    %calculate and update downforce, drag and rolling resistance acting on the car 
    

    %downforce pushing car down and increasing grip 
    downforce(pos) = ((1/2) * cdf * V(pos)^2 * frontal_A * rho); 
    
    %drag counteracting car forward movement 
    drag(pos) = ((1/2) * cd * V(pos)^2 * frontal_A * rho); 

    %rolling resistance 
    rollr(pos) = 0.02 * (mass * g + downforce(pos)); 

    %longitiduinal weight transfer occuring as a result of acceleration 
    WTlong(pos) = 0.5* mass * A(pos) * (COG_height / wheelbase); %weight transfer 
    
    %rear grip considering load transfer during acceleration (both tires)
    Fgriprear(pos) = mux * ((mass * g * rear_weight_distribution)+(downforce(pos) * Cp) + WTlong(pos));
    %front grip considering load transfer during acceleration (both tires)
    Fgripfront(pos) = mux * ((mass * g * (1-rear_weight_distribution))+(downforce(pos) * (1-Cp)) - WTlong(pos));

    %available motor torque for rear motor from grip on ground (one tire)
    rear_friction_torque(pos) = Fgriprear(pos) * tire_radius / gearratio / num_motors / gearbox_e; 

    %available torque for front motors from grip on ground (one tire)
    front_friction_torque(pos) = Fgripfront(pos) * tire_radius / frontgearratio / num_motors / gearbox_e; 

    if T(pos) > 1.24
        derate = 1; 
    end

    %plug values into motor simulation function. function outputs torque
    %for front and rear motors, power usage, current, voltage etc 

    [rear_motor_power_consumption(pos), front_motor_power_consumption(pos), rear_motor_torque(pos), front_motor_torque(pos), ...
     rear_voltage_line_RMS(pos), front_voltage_line_RMS(pos), rear_stator_current_line_RMS(pos), front_stator_current_line_RMS(pos), rear_I_qs(pos), front_I_qs(pos), rear_I_ds(pos), front_I_ds(pos), battery_current(pos), inverter_DCV(pos)] ...
     = motor_function(power_limit, derate, rear_bias, rear_motor_rpm(pos), gearratio, front_motor_rpm(pos), frontgearratio,rear_friction_torque(pos), front_friction_torque(pos), inverter_e, interpolatedTorque, ...
     interpolatedVoltage, interpolatedCurrent, interpolatedPowerConsumption, interpolatedPowerFactor, interpolatedIqRMS, current_fine, battery_OCV, battery_resistance);

    %calculate force from motors pushing car forward. 

    forcerear(pos) = rear_motor_torque(pos) * gearratio * num_motors * gearbox_e / tire_radius; 

    forcefront(pos) = front_motor_torque(pos) * frontgearratio * num_motors * gearbox_e / tire_radius; 
    
    %removes extra data point that makes graph look weird (comes from
    %dividing by zero) 

    if(rear_bias == 1.0)
    forcefront(pos) = 0; 
    end    
  

    %calculated the total force pushing the car forward minus the forces counteracting the car  

    Force(pos) = forcerear(pos) + forcefront(pos) - drag(pos) - rollr(pos);  
 
    %Update kinematics
    
    A(pos+1) = Force(pos)/mass; 
    %rev limiter  %pretty sure this is not valid 
    if rear_motor_rpm(pos) > max_rpm || front_motor_rpm(pos) > max_rpm
    
     A(pos+1) = 0; %car cannot accelerate further once max rpm is reached 

    end 

    % power limiter. hitting 80 kW power limit = no more acceleration 
    if battery_current(pos) * battery_OCV >= max_power

        power_limit = 1; 

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

%% MOTOR EFFICIENCY AND HEAT GENERATION
motor_power_consumption = rear_motor_power_consumption+front_motor_power_consumption;
mechanical_output_power = (rear_motor_torque .* ((2 * pi * rear_motor_rpm / 60) / 1000)) + (front_motor_torque .* ((2 * pi * front_motor_rpm / 60) / 1000));%kW


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


% %Display 75 m acceleration time 
fprintf("75 m acceleration event time: %0.2f for overall gear ratio: %0.1f\n", T(pos+1), frontgearratio); 
% Display max battery power consumption (for purpose of finding optimal
% motor current parameters) 
fprintf("Top Speed During Accel: %0.1f m/s for overall gear ratio: %0.1f\n", max(V), frontgearratio); 

%% Gear Ratio Sweep 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf("75 m acceleration event time: %0.2f for power split: %0.2f\n", T(pos+1), rear_bias); 
Acceltime(i) = T(pos+1); 
maxpower(i) = round(max(battery_power_consumption)); 
topspeed(i) = max(V);
maxpowerdelta(i) = (maxpower(i) - 80)/80; 
i=i+1; %counter variable 
end
% 
% Display figure for front gear ratio sweep

figure(1); 

yyaxis("left"); 
plot(gr_sweep, Acceltime, 'bo'); 
ylabel("Time (s)"); 

yyaxis("right"); 
plot(gr_sweep, topspeed, 'ro'); 
ylabel("Top Speed (m/s)"); 


legend("Time", "Top Speed");
title("75m acceleration event time and top speed for overall gear ratio 6 - 18"); 
xlabel("Gear Ratio");

grid on; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %% Display figure for front-rear power split sweep 
% 
% figure(1); 
% plot(rb_sweep, Acceltime, 'bo'); 
% title("75m acceleration event time for front-rear power split"); 
% xlabel("Power split (fraction of power to rear motors)");
% ylabel("Time (s)"); 
% grid on; 


%% Display single accel sim forces and traction graphs 

%plot AVP Graph 

fprintf("75 m acceleration event time: %0.2f\n", T(pos+1)); 

figure(6)

subplot(3,1,1); 

plot(T,AccelG, 'ro'); 

title("Acceleration vs Time"); 
xlabel("Time (seconds)"); 
ylabel("Acceleration (gs)"); 
grid on

subplot(3,1,2); 

plot(T, V, 'go'); 

title("Velocity vs Time"); 
xlabel("Time (seconds)"); 
ylabel("Velocity (m/s)"); 
grid on 

subplot(3,1,3); 

plot(T, P, 'bo'); 

title("Position vs Time"); 
xlabel("Time (seconds)"); 
ylabel("Position (m)"); 
grid on



figure(2); 
hold on
plot(T, forcerear, 'bo'); 
plot(T, Fgriprear, 'ro'); 
title("Force & Grip from rear wheels vs Time"); 
xlabel("Time (seconds)"); 
ylabel("Force (N)"); 
legend("Available Power", "Available Traction"); 
grid on
hold off
figure(3); 
hold on
plot(T, forcefront, 'bo'); 
plot(T, Fgripfront, 'ro'); 
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
figure(4)
subplot(4, 1, 1)
scatter(rear_motor_rpm, rear_motor_torque, "red")
title("Acceleration Run [75m]")
xlabel("Motor RPM")
ylabel("Motor Torque [Nm]")
subtitle("Torque/RPM Data")
hold on 

scatter(rear_motor_rpm, front_motor_torque, "blue")

scatter(rear_motor_rpm, front_motor_torque+rear_motor_torque, "green")

hold off

legend("Rear motor torque", "Front motor torque", "Combined Torque"); 

grid on

subplot(4, 1, 2)
scatter(rear_motor_rpm, rear_stator_current_line_RMS,"red")
xlabel("Motor RPM")
ylabel("Current RMS [A]")
hold on 
title("Current data: rear motor");

scatter(rear_motor_rpm, rear_I_qs, "blue")

scatter(rear_motor_rpm, rear_I_ds, "yellow"); 

legend("Stator Current", "Torque-Producing Stator Current, Iq", "field weakening current, Id");
grid on
hold off 

subplot(4, 1, 3); 
hold on
title("Power consumption & loss"); 
plot(rear_motor_rpm, battery_power_consumption, 'ro'); 
plot(rear_motor_rpm, total_motor_power_consumption, 'bo'); 
plot(rear_motor_rpm, battery_heat_generation/1000, 'go');
legend("Battery Power Consumption", "Motor Power Consumption", "Battery Heat losses"); 
xlabel("time (sec)"); 
ylabel("power consumption (kw)"); 
grid on 

fprintf("Maximum battery power consumption: %0.2fkW\n", max(battery_power_consumption)); 

subplot(4, 1, 4)
scatter(rear_motor_rpm, motor_power_consumption)
grid on
xlabel("Motor RPM")
ylabel("Power [kW]")
title("Power vs RPM")
hold on
scatter(rear_motor_rpm, mechanical_output_power*1000)
legend("Electrical Input", "Mechanical Output")
grid on
hold off

% more motor / battery characteristics 

figure(5)

subplot(3, 1, 1)
hold on

scatter(rear_motor_rpm, rear_voltage_line_RMS, "red")
scatter(rear_motor_rpm, front_voltage_line_RMS, "blue")
scatter(rear_motor_rpm, inverter_DCV / sqrt(2), "green")
title("Voltage Data")
legend("Rear voltage line RMS", "Front voltage line RMS", "Inverter DCV / root 2")

xlabel("Motor RPM")
ylabel("Voltage [V]")
grid on
hold off 

subplot(3,1,2) 

scatter(rear_motor_rpm, battery_current)

title("Battery current")

xlabel("Motor RPM"); 
ylabel("Current (A)"); 
grid on

subplot(3,1,3) 

scatter(rear_motor_rpm, rear_stator_current_line_RMS, "red") 
hold on
scatter(rear_motor_rpm, front_stator_current_line_RMS, "blue")
legend("Rear Motor Current", "Front Motor Current"); 
title("Commanded Current")
hold off

%% Correlation plots %% 

m = readmatrix("decoded_data(5).csv"); 

startp = 10861.5/0.015;

endp = 10865/0.015; 




torquedata = m(startp:endp,16); 
signal = m(startp:endp,1); 
rpmdata = m(startp:endp,17); 


t = 0:0.015:(size(torquedata,1)-1)*0.015; 

velocitydata = rpmdata / 60 / 11.33 * 0.2 * 2 * 3.141;





figure(7) 
sgtitle("Acceleration Run [75m]: Test Data vs Simulated")
subplot(3,1,3)
hold on
plot(T, rear_motor_torque, "b-")
scatter(t',torquedata/10); 
legend("simulated", "test");
xlabel("Time [s]")
ylabel("Motor Torque [Nm]")
grid on
hold off
subplot(3,1,2) 
hold on
plot(T, rear_motor_rpm, "b-") 
scatter(t',rpmdata)
legend("simulated", "test");

xlabel("Time [s]")
ylabel("Motor rpm")  
grid on
hold off
subplot(3,1,1) 
hold on
plot(T, V,"b-")
scatter(t', velocitydata); 
legend("simulated", "test");

xlabel("Time [s]")
ylabel("Velocity [m/s]")  
hold off
grid on 
