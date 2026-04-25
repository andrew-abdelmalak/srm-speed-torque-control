%% SRM Parameters — 6/4 Three-Phase Machine
Rs    = 1.0;         % Phase resistance [Ohm]
Lmin  = 0.020;       % Minimum inductance [H] (unaligned)
Lmax  = 0.150;       % Maximum inductance [H] (aligned)
Nph   = 3;           % Number of phases
Nr    = 4;           % Number of rotor poles
Ns    = 6;           % Number of stator poles

%% Angular positions [radians] — 6/4 machine
pole_pitch   = 2*pi/Nr;          % 90 deg = pi/2
phase_offset = 2*pi/(Nph*Nr);    % 30 deg
theta_on     = 0;                % Turn-on angle (relative to unaligned)
theta_off    = deg2rad(38);      % Turn-off angle
theta_ov     = deg2rad(10);      % TSF overlap angle

%% Mechanical parameters
J  = 0.002;   % Moment of inertia [kg.m^2]
B  = 0.001;   % Viscous friction [N.m.s/rad]
TL = 1.5;     % Load torque [N.m]

%% Converter
Vdc  = 300;   % DC link voltage [V]
Imax = 15;    % Peak current limit [A]

%% Control — Speed PI
Kp_w = 0.5;
Ki_w = 10;
T_max = 5;   % Max torque reference [N.m]

%% Control — Current PI (for PWM option)
Kp_i = 2.0;
Ki_i = 500;

%% Simulation
Ts   = 1e-5;   % Sample time [s] (100 kHz for hysteresis)
Tsim = 0.5;    % Total simulation time [s]
delta_i = 0.1; % Hysteresis band [A]

%% Generate Inductance LUT Data 
% Create an angle vector from 0 to 90 degrees (one rotor pole pitch)
theta_vec = linspace(0, pi/2, 1000); 
L_vec = zeros(size(theta_vec));

% Build the piecewise linear profile
for k = 1:length(theta_vec)
    th = theta_vec(k);
    if th < theta_on
        L_vec(k) = Lmin;
    elseif th >= theta_on && th < theta_off
        % Rising inductance slope
        L_vec(k) = Lmin + (Lmax - Lmin) * ((th - theta_on) / (theta_off - theta_on));
    elseif th >= theta_off && th < (theta_off + deg2rad(5))
        % Small flat top at maximum inductance (fully aligned)
        L_vec(k) = Lmax;
    else
        % Falling inductance slope (symmetric to rising)
        th_falling_start = theta_off + deg2rad(5);
        th_falling_end = th_falling_start + (theta_off - theta_on);
        if th < th_falling_end
            L_vec(k) = Lmax - (Lmax - Lmin) * ((th - th_falling_start) / (th_falling_end - th_falling_start));
        else
            L_vec(k) = Lmin;
        end
    end
end

% Calculate the derivative dL/dθ
dLdth_vec = gradient(L_vec, theta_vec);
tsf_type = 3;  % TSF Profile: 1=Linear, 2=Sinusoidal, 3=Cubic, 4=Exponential