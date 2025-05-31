function [Z_arr v_meas_arr] = getMeas2(X_arr, Z_arr, index, v_meas_arr, vStd, dt)
	
	%% Constants
	ET = 256;									% Total encoder ticks in one rotation
	R = 0.05;									% Radius of wheels
	
	%% Creating sensor measurements
	l = X_arr(1, index) - X_arr(1, index-1);	% True length traveled since last step
	l = l*ones(1,3) +  dt * vStd * randn(1,3);		% Lengths travelled by all 3 wheels + noise
	ticks = round((l*ET) / (2*pi*R));			% Amount of ticks counted by each encoder
	
	%% Extracting data from measurements
	valid_meas = 3;
	tick_meas = ticks;
	ticks_counted = sum(tick_meas)/valid_meas;	% Averaging the valid measurements
	l_meas = (ticks_counted*2*pi*R) / ET;		% Measured length by encoder
	pos_meas = Z_arr(1, index-1) + l_meas;		% Measured position
% 	
	%% Running low pass filter for velocity measurements
    w = 0.9;
	v_meas = (1-w)*l_meas/dt + w*v_meas_arr(1, index-1);							% Without any filtering
    v_meas_arr(index) = v_meas;
    
	%% Return Z, the measurement array
	Z_arr(:,index) = [pos_meas; v_meas];
end