function [Z_arr v_meas_arr] = getMeas(X_arr, Z_arr, index, v_meas_arr, vStd, dt)
	
	%% Constants
	ET = 256;									% Total encoder ticks in one rotation
	R = 0.05;									% Radius of wheels
	likelihood_of_slipping = 0.1;				% Likelihood of a wheel slipping
	
	%% Creating sensor measurements
	l = X_arr(1, index) - X_arr(1, index-1);	% True length traveled since last step
	l = l*ones(1,3) + dt * vStd * randn(1,3);    % Lengths travelled by all 3 wheels + noise
	ticks = round((l*ET) / (2*pi*R));			% Amount of ticks counted by each encoder
	
% 	for i = 1:3
% 		if rand < likelihood_of_slipping		% In case the wheel slips
%             ticks(i) = ticks(i) * rand * 0.5;	% The encoder detects much fewer ticks
% 		end
% 	end
	
	%% Extracting data from measurements
	valid_meas = 3;
	tick_meas = ticks;
	for i = 1:3									% Removing outliers / SLIP DETECTION
		if abs(tick_meas(i))<abs(0.2*sum(ticks))% If a measurement contains too few ticks
			tick_meas(i) = 0;					% Set the measurement to 0
			valid_meas = valid_meas-1;			% One less valid measurement
		end
	end
	ticks_counted = sum(tick_meas)/valid_meas;	% Averaging the valid measurements
	l_meas = (ticks_counted*2*pi*R) / ET;		% Measured length by encoder
	pos_meas = Z_arr(1, index-1) + l_meas;		% Measured position

	v_meas = l_meas/dt;							% Without any filtering
	%% Return Z, the measurement array
	Z_arr(:,index) = [pos_meas; v_meas];
end