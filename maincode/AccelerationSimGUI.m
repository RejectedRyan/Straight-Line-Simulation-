    
function AccelerationSimGUI
     
    close all
    % Create UI figure
    fig = uifigure('Name','Bicycle Model Acceleration Simulation','Position',[100 100 420 320]);

    % Dropdown for drivetrain
    uilabel(fig,'Position',[20 260 150 22],'Text','Select Configuration:');
    dd = uidropdown(fig, ...
        'Items',{'RWD','AWD'}, ...
        'ItemsData',[1.0, 0.0], ... % 1.0 = RWD, 0.0 = AWD
        'Position',[180 260 120 22]);
   
    % Gear ratio low
    uilabel(fig,'Position',[20 210 150 22],'Text','Low Gear Ratio:');
    grLowEdit = uieditfield(fig,'numeric', ...
        'Value',11.33, ...
        'Position',[180 210 120 22]);

    % Gear ratio high
    uilabel(fig,'Position',[20 160 150 22],'Text','High Gear Ratio:');
    grHighEdit = uieditfield(fig,'numeric', ...
        'Value',11.33, ...
        'Position',[180 160 120 22]);
   
    % Mass
    uilabel(fig,'Position',[20 110 150 22],'Text','Vehicle Mass (kg):');
    massEdit = uieditfield(fig,'numeric', ...
        'Value' ,290, ...
        'Position',[180 110 120 22]);

    % Run button
    uibutton(fig,'push', ...
        'Text','Run Simulation', ...
        'Position',[140 40 140 40], ...
        'ButtonPushedFcn',@(btn,event) runSimulation(dd,grLowEdit,grHighEdit, massEdit));
end

function runSimulation(dd,grLowEdit,grHighEdit,massEdit)
    % Collect parameters from GUI
    rear_bias = dd.Value;
    gr_low = grLowEdit.Value;
    gr_high = grHighEdit.Value;
    mass0 = massEdit.Value;                      % kg baseline

    % Baseline values
    cd0 = 1.28;                       % coefficient of drag
    cdf0 = 3.75;                      % coefficient of downforce
    rear_weight_distribution0 = 0.57; % percent mass in rear
    mux0 = 1.4;                       % average tire friction coefficient
    
    sensitivity = 0;   % Toggle for sensitivity study
    percent = 10;      % percent variation for sensitivity study
    
    if sensitivity
        % Run baseline first
        baseline = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0,cd0,cdf0,rear_weight_distribution0,mux0);
        
        % Factor for ±% changes
        factor = percent/100;
        
        % Sensitivity for Mass
        mass1 = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0*(1+factor),cd0,cdf0,rear_weight_distribution0,mux0)
        mass2 = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0*(1-factor),cd0,cdf0,rear_weight_distribution0,mux0)

        % Sensitivity for Cd
        cd1 = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0,cd0*(1+factor),cdf0,rear_weight_distribution0,mux0)
        cd2 = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0,cd0*(1-factor),cdf0,rear_weight_distribution0,mux0)

        % Sensitivity for Cdf
        cdf1 = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0,cd0,cdf0*(1+factor),rear_weight_distribution0,mux0)
        cdf2 = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0,cd0,cdf0*(1-factor),rear_weight_distribution0,mux0)

        % Sensitivity for Rear Weight Distribution
        rw1 = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0,cd0,cdf0,rear_weight_distribution0*(1+factor),mux0)
        rw2 = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0,cd0,cdf0,rear_weight_distribution0*(1-factor),mux0)

        % Sensitivity for Mux
        mux1 = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0,cd0,cdf0,rear_weight_distribution0,mux0*(1+factor))
        mux2 = bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0,cd0,cdf0,rear_weight_distribution0,mux0*(1-factor))
        
    else
        % Just run once with given values
        bicyclemodel4WDsim(rear_bias,gr_low,gr_high,mass0,cd0,cdf0,rear_weight_distribution0,mux0, 1);
    end
end
