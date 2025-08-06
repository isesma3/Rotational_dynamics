%% PROJECT  A -- AE6520 -- SPRING 2025
% author: Ismael Rodriguez Sesma

clc; clear; close all
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot,'defaultLineLineWidth', 2)

%% INPUTS
% Rigid body geometry [geometry units]
LENGTH = 8;
WIDTH = 5;
THICK = 2;

% For moments of inertia assumed mass is 1 kg
i_xx = (WIDTH^2 + THICK^2)/12;
i_yy = (LENGTH^2 + THICK^2)/12;
i_zz = (WIDTH^2 + LENGTH^2)/12;

% Integration data
INTEG_TIME = 300;    % [s]
INTEG_STEP = 1E-2;    % [s]
tSpan = linspace(0, INTEG_TIME, INTEG_TIME/INTEG_STEP);

%% PERTURBATION
% Initial condition at 0 seconds
caseName = 'Intermediate Axis';

% Change the inertia components
switch caseName
    case 'Minor Axis', inertMom = [i_yy, 0, 0; 0, i_zz, 0; 0, 0, i_xx];
    case 'Intermediate Axis', inertMom = [i_zz, 0, 0; 0, i_xx, 0; 0, 0, i_yy];
    case 'Major Axis', inertMom = [i_xx, 0, 0; 0, i_yy, 0; 0, 0, i_zz];
end

% Initial perturbation at 0+ seconds
PHI_0 = 0.2;     % Initial perturbation arount the inertial frame [rad]
initOmega = [0 0 0.1]';

pertRotMatrix = dcm_matrix(PHI_0, 0, 0);    % From Inertial to body
omegaBody = inertMom\(pertRotMatrix*inertMom*initOmega);    % Initial ang velocity  of body frame wrt inertial frame [rad/s] 

%% ANGULAR VELOCITY, EULER ANGLES AND QUEATERNION INTEGRATION
initAttAng = [PHI_0 0 0];
initQuat = [cos(PHI_0/2); sin(PHI_0/2); 0; 0];

initCond = [omegaBody; initQuat];

% Angular velocity on body frame
[~, y] = ode45(@(t, y) rot_rigid_ode(t, y, inertMom), tSpan, initCond);
omegaEvol = y(:, 1:3);      % [rad/s]
omegaEvolAng = rad2deg(omegaEvol);      % [deg/s]
quatEvol = y(:, 4:7);

eulerAngEvol = euler_quat(quatEvol);

% Check that angular momentum is conserved
initAngMom = norm(inertMom*omegaBody);
finalAngMom = norm(inertMom*omegaEvol(end, :)');

if (finalAngMom - initAngMom) < abs(1E-2)
    disp('Angular momentum is conserved');
else
    warning('Angular momentum is not conserved and it should!');
end

% Attitude angles
[t, y] = ode45(@(t, y) attitude_ode(t, y, omegaEvol, tSpan), tSpan, initAttAng);
attAngEvol = y;     % [rad]
attAngEvol(:, 3) = mod(attAngEvol(:, 3) + pi, 2*pi) - pi;       % Fix Yaw rate to be contained between - pi and pi

attAngEvo = rad2deg(attAngEvol);
eulerAngEvo = rad2deg(eulerAngEvol);

%% PLOTS
% Plot angular velocity components
figure(1);
plot(t, omegaEvolAng(:, 1), 'b', t, omegaEvolAng(:, 2), 'r', t, omegaEvolAng(:, 3), 'g');
xlabel('Time [s]');
ylabel('Angular Velocity [deg/s]');
legend('$r$', '$p$', '$q$');
title('Angular Velocity vs Time');
grid on;

% Plot attitude angles evolution computed with angular velocity
figure(2);
plot(t, attAngEvo(:, 1), 'b', t, attAngEvo(:, 2), 'r', t, attAngEvo(:, 3), 'g');
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('$\psi$', '$\phi$', '$\theta$');
title('Euler angles with Euler kinematic equations');
grid on;

% Plot attitude angles evolution computed with quaternions
figure(3);
plot(t, eulerAngEvo(:, 1), 'b', t, eulerAngEvo(:, 2), 'r', t, eulerAngEvo(:, 3), 'g');
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('$\psi$', '$\phi$', '$\theta$');
title('Euler angles with quaternion formulation');
grid on;

% EOF
