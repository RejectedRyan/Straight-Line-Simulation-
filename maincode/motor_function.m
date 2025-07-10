function [rear_motor_power_consumption, front_motor_power_consumption, rear_motor_torque, front_motor_torque, rear_voltage_line_RMS, front_voltage_line_RMS, rear_stator_current_line_RMS, front_stator_current_line_RMS, rear_I_qs, front_I_qs, rear_I_ds, front_I_ds, battery_current, inverter_DCV] = motor_function(derate, rear_bias, rear_motor_rpm, gearratio, front_motor_rpm, frontgearratio, rear_friction_torque, front_friction_torque, inverter_e, interpolatedTorque, interpolatedVoltage, interpolatedCurrent, interpolatedPowerConsumption, interpolatedPowerFactor, interpolatedIq, battery_OCV, battery_resistance)
%% RATED POWER
% This function determines the motor parameters based upon the state of motor RPM, 
% Traction, Battery Voltage and Resistance using the interpolated motor
% data. The motor can operate in 4 conditions.
% 1) traction limited, 2) power limited by the motor itself, 3)Limited by the battery


%% DEFINE RPM STATE

% Determine motor table row location based upon RPM
% add + 1 because MATLAB is 1 based (0 RPM is in the 1st row)
table_row_location_rear = round(rear_motor_rpm) + 1;
table_row_location_front = round(front_motor_rpm) + 1; 




%% CONSTANT TORQUE REGION 
% TRACTION LIMITED OR POWER/MOTOR LIMITED STATE
% This section calculates the motor torque based upon the motor's
% capability (power) at the given RPM and the avaiable traction at the tire contact
% patch. The motor torque is set as the minimum torque. The other motor
% parameters are determined based upon the RPM state and calculated torque 

% ------------------ Determine Motor Torque ---------------------------
% Determine motor torque based upon given rpm state, friction at tire contact patch, and maximum motor current bounded by thermals


%max current should changed based on amount of rear bias. need to talk to 
%battery people 

%Also, we do a programmed derating at a certain point, iirc 1.24 seconds.
% this theoretically will allow higher max currents without breaking the 80
% kW power limit. 

if rear_bias == 1.0  % this part doesn't really make sense, predicts a accel time of 5+ sec which def seems too low. 
    if derate 
    rear_max_current_accel = 85;  % [A] This is the maximum current that can be applied for 5s before the motor overheats
    front_max_current_accel = 0; 

    else
    rear_max_current_accel = 105; %This is the peak current the motor can utilize
    front_max_current_accel = 0; 
    end

else 
    
    if derate
    rear_max_current_accel = 60 * (11.33/(gearratio));  %using 25 - 75 power split between front and rear, following derating. 
    front_max_current_accel = 20 * (11.33/(frontgearratio));  %these numbers need to be subject to change based on the gear ratio. otherwise we will exceed the power limit 
    %fprintf("%0.1f\n", rear_max_current_accel); 
    else
    rear_max_current_accel = min(105, 105 * (11.33/(gearratio))); % [A] This is the maximum current that can be applied for 1.24s before the motor overheats
    front_max_current_accel = 55 * (11.33/(frontgearratio)); % This is approximately the maximum current given the grip limitations of the front wheels 
    end

end

rear_stator_current_column = round(rear_max_current_accel / 0.1) + 1; % column location for the maximum current that can be applied to the rear motor 
rear_torque_vals = interpolatedTorque(table_row_location_rear,1:rear_stator_current_column); % torque values at current rpm
rear_torqueMAX_atRPM = max(rear_torque_vals); % maximum torque that motor can output at current rpm

front_stator_current_column = round(front_max_current_accel / 0.1) + 1; %column location for max current that can be applied for the front motor 
front_torque_vals = interpolatedTorque(table_row_location_front, 1:front_stator_current_column); %torque values for current rpm, for front motor 
front_torqueMAX_atRPM = max(front_torque_vals); %maximum torque that the front motors can output at current rpm 

% The motor is either limited by traction or the motor itself

rear_motor_torque = min(rear_torqueMAX_atRPM, rear_friction_torque); % rear motor torque [Nm]
front_motor_torque = min(front_torqueMAX_atRPM, front_friction_torque); % front motor torque [Nm]  


% ------------------ Tabulated Data Location ---------------------------
% Determone column location of the commanded motor torque in the interpolatedTorque table
% This basically finds the closest torque value in the interpolatedTorque table 
% finidng this location in the table allows us to find the numerical value
% of other motor parameters
%    rear 
locate_column_vector_rear = abs(rear_torque_vals - rear_motor_torque);
column_vector_min_rear = min(locate_column_vector_rear);
table_column_location_rear = locate_column_vector_rear == column_vector_min_rear;

%    front

locate_column_vector_front = abs(front_torque_vals - front_motor_torque); 
column_vector_min_front = min(locate_column_vector_front); 
table_column_location_front = locate_column_vector_front == column_vector_min_front; 


% ------------------ Determine Motor Voltage, Current, and Electric Power based on commanded torque values ---------------------------

%----rear------
%fprintf("%i", table_column_location_rear); 
%fprintf("%0.3f ", table_row_location_rear); 
rear_motor_power_consumption = interpolatedPowerConsumption(table_row_location_rear, table_column_location_rear);
rear_voltage_line_RMS = interpolatedVoltage(table_row_location_rear, table_column_location_rear);
rear_stator_current_line_RMS = interpolatedCurrent(table_row_location_rear, table_column_location_rear);
rear_I_qs = sqrt(3) * interpolatedIq(table_row_location_rear, table_column_location_rear);
rear_I_ds = sqrt(rear_stator_current_line_RMS^2 - rear_I_qs^2); % magnetizing stator current - this current is caused by Vd which is in opposition to BEMF


%-----front---- 

front_motor_power_consumption = interpolatedPowerConsumption(table_row_location_front, table_column_location_front);
front_voltage_line_RMS = interpolatedVoltage(table_row_location_front, table_column_location_front);
front_stator_current_line_RMS = interpolatedCurrent(table_row_location_front, table_column_location_front);
front_I_qs = sqrt(3) * interpolatedIq(table_row_location_front, table_column_location_front);
front_I_ds = sqrt(front_stator_current_line_RMS^2 - front_I_qs^2); % magnetizing stator current - this current is caused by Vd which is in opposition to BEMF


% ------------------ Battery and Inverter Current, Voltage, and Power ---------------------------
% note: consumption = power in != power out

%power consumption for both motors added together 

total_inverter_power_consumption = (rear_motor_power_consumption + front_motor_power_consumption)/inverter_e; 


%battery power out = rear interverter power + front inverter power * 2 

battery_power_out = total_inverter_power_consumption * 2; 


a = battery_resistance;
b = -battery_OCV;
c = battery_power_out; 

d = (front_motor_power_consumption/inverter_e) * 2; 
% Total power consumption for both motors

%battery current will increase as power consumption increases, so it should
%be higher in 4wd, but only slighly 
%likewise, drop DCV increses, causing field weakening to come sooner. but
%that does make sense, because more power is being drawn either way. 

battery_current = (-b - sqrt(b^2 - 4*a*c)) / (2*a);
front_battery_current_contribution = (-b - sqrt(b^2 - 4*a*d)) / (2*a);
drop_DCV = battery_current * battery_resistance;
inverter_DCV = battery_OCV - drop_DCV;

%in non field weakening, inverter DCV should be the same for both motors 



rear_fw = 0; 
front_fw = 0; 

%Field weakening takes place to allow a motor to operate at higher rpm
%levels that its base/rated speed. As a motor spins faster, there is an
%increased level of back EMF, in our case voltage line RMS. Once the
%voltage line RMS reaches the inverter voltage, the motor cant spin any
%faster while maintaining torque, so it injects Id (field weakening
%current) that reduces the stator line current, keeping the voltage line RMS at the same
%level as rpm increases. 

%Field weakening can occur at different times for the front and rear
%motorsif they have different max stator line current, because the voltage
%line rms rises along with current. 

%as a result, there are 3 new cases: Both motors in constant torque, rear
%in field weakening, front constant torque, and both in field weakening

% Essentially what occurs in this section is that we are finding the maximum motor
% torque torque that is available given the rpm, and inverter voltage. As
% rpm continues to increase, the current/torque must decrease so that the
% vrms does not surpass the inverter voltage. 

%inverter voltage is found using the process of V = IR + Vo where I is the
%battery current, R is the cell resistance and Vo is the original voltage 

%% FLUX WEAKENING ("CONSTANT POWER REGION")

% ------------------ Check if rear motor is in field weakening region ---------------------------
if rear_voltage_line_RMS > inverter_DCV / sqrt(2) %sqrt is to convert from RMS to DC 

    %fprintf("\n Rear Motors in field weakening"); 

    rear_fw = 1; 

  
    % ------------------Set stator current rms to maximum possible value for particular straight---------------------------

    % -- rear -- 
    
    rear_stator_current_line_RMS = rear_max_current_accel; % A
    rear_power_factor = interpolatedPowerFactor(table_row_location_rear, rear_stator_current_column);



    %---------------- Generate vectors based upon motor electric power with specifed stator current RMS ------------------------
    
    %should be calculated at the same time 

    rear_motor_electric_power_vector = sqrt(3) * rear_stator_current_line_RMS .* rear_power_factor .* interpolatedVoltage(table_row_location_rear, 1:rear_stator_current_column); % Watts
    rear_inverter_power_vector =  rear_motor_electric_power_vector / inverter_e; % Watts
    rear_battery_power_out_vector = rear_inverter_power_vector * 2; % Watts
   % fprintf("%i\n", size(rear_battery_power_out_vector));

end 

% --- check if front motor is in field weakening region  --- 
 
if front_voltage_line_RMS > inverter_DCV / sqrt(2) 
    %fprintf("\n Front Motors in field weakening"); 
    front_fw = 1; 
    % -- front -- 

    front_stator_current_line_RMS = front_max_current_accel; %A  
    front_power_factor = interpolatedPowerFactor(table_row_location_front, front_stator_current_column); 


    %---------------- Generate vectors based upon motor electric power with specifed stator current RMS ------------------------
    
    %should be calculated at the same time as rear 



    front_motor_electric_power_vector = sqrt(3) * front_stator_current_line_RMS .* front_power_factor .* interpolatedVoltage(table_row_location_front, 1:front_stator_current_column); % Watts
    front_inverter_power_vector =  front_motor_electric_power_vector / inverter_e; % Watts
    front_battery_power_out_vector = front_inverter_power_vector * 2; % Watts

    %adjust length of front battery power vector to match that of rear
    %vector. 


end


if rear_fw 

    if front_fw
   
    c = front_battery_power_out_vector; 
    front_battery_current_vector = (-1 * (-4*a*c + b^2).^(1/2) - b) / (2*a); % A
    % battery_power_consumption_vector = battery_current_vector * battery_OCV;
    %front_inverter_DCV_vector = -front_battery_current_vector * battery_resistance + battery_OCV; 
    
    %creates vector used to adjust front battery current so that it can be
    %added to rear 
    front_current_vector_adj = zeros(1, size(rear_battery_power_out_vector, 2) - size(front_battery_power_out_vector, 2)) + front_battery_current_vector(1, size(front_battery_current_vector,2));

    %now front and rear should be the same size     

    front_battery_current_vector = [front_battery_current_vector, front_current_vector_adj]; 
    else 

    %basically just adds non field weakening power usage front front motors, in case that they haven't reached field weakening yet 

    front_battery_current_vector = front_battery_current_contribution; 
    
    end 
    
    c = rear_battery_power_out_vector; 
    rear_battery_current_vector = (-1 * (-4*a*c + b^2).^(1/2) - b) / (2*a); % A
    % battery_power_consumption_vector = battery_current_vector * battery_OCV;
    

  
    
    %calculate inverter DCV vector based on combined battery current
    %vectors. need to adjust size of battery current vectors for front 
    
    battery_current_vector = rear_battery_current_vector + front_battery_current_vector;
    
    
    inverter_DCV_vector = -battery_current_vector * battery_resistance + battery_OCV; 


    % now, to use inverter dcv vector for front, need to adjust size again?

    front_inverter_DCV_vector = inverter_DCV_vector(1, 1:front_stator_current_column);

    % Did something interesting here (unsure if correct) to calculate
    % inverterDCV. Calculated battery currents seperately for front and
    % rear and added them together, but since the vectors for the front
    % motors are smaller, to add them I extended the front vector with so
    % that it would repeat its last value. 


    % ------------------- Find DC bus voltage and voltage line RMS  -----------------
    
    %use the combined inverter values to calculate individual values for
    %the front and rear 
    %-- rear--
    findVoltageState = abs(inverter_DCV_vector/sqrt(2) - interpolatedVoltage(table_row_location_rear, 1:rear_stator_current_column));
    minVoltageDiff = min(findVoltageState);
    rear_VRMS_column = findVoltageState == minVoltageDiff; %returns the column of voltage that is closest to the found inverter DCV 



    rear_voltage_line_RMS = interpolatedVoltage(table_row_location_rear, rear_VRMS_column);
   
    
    if front_fw

    front_findVoltageState = abs(front_inverter_DCV_vector/sqrt(2) - interpolatedVoltage(table_row_location_front, 1:front_stator_current_column));
    front_minVoltageDiff = min(front_findVoltageState);
    front_VRMS_column = front_findVoltageState == front_minVoltageDiff;



    front_voltage_line_RMS = interpolatedVoltage(table_row_location_front, front_VRMS_column);
    
    
    
    end

    inverter_DCV = max(rear_voltage_line_RMS * sqrt(2), front_voltage_line_RMS * sqrt(2)); 


    %-------------------- Find torque-producing stator current and motor torque ---------------------
    rear_motor_torque = interpolatedTorque(table_row_location_rear, rear_VRMS_column);
    rear_I_qs = sqrt(3) * interpolatedIq(table_row_location_rear, rear_VRMS_column); % torque-producing stator current 
    rear_I_ds = sqrt(rear_stator_current_line_RMS^2 - rear_I_qs^2); % magnetizing stator current - this current is caused by Vd which is in opposition to BEMF
    
    if front_fw
    front_motor_torque = interpolatedTorque(table_row_location_front, front_VRMS_column); 
    front_I_qs = sqrt(3) * interpolatedIq(table_row_location_front, front_VRMS_column); % torque-producing stator current 
    front_I_ds = sqrt(front_stator_current_line_RMS^2 - front_I_qs^2); % magnetizing stator current - this current is caused by Vd which is in opposition to BEMF
    end 

    


    % ------------------------- Motor Electric Power -------------------------
    rear_motor_power_consumption = sqrt(3) * rear_stator_current_line_RMS * rear_power_factor * rear_voltage_line_RMS;
    if front_fw
    front_motor_power_consumption = sqrt(3) * front_stator_current_line_RMS * front_power_factor * front_voltage_line_RMS; 
    end
    total_motor_power_consumption = (rear_motor_power_consumption * 2) + (front_motor_power_consumption * 2); 

  
    
    % --------------------- Find Battery Current ---------------------
    battery_current = total_motor_power_consumption / inverter_e / inverter_DCV; 

end 

    
