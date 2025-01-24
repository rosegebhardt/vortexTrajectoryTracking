% This file stores system parameters needed for each file.

% System parameters
omega = 2.4; % frequency (rad/sec)
aoa = 0; % AOA (rad)
Gamma_v = 1; % vorticity (rad/sec)
delta0 =  -0.2; % thickness parameter with zero camber (unitless)
r0 = 1; % cylinder radius (m)
u0 = 2; % freestream velocity (m/s)
k0 = r0*(delta0 + 1); % uncambered thickness (unitless)
A = sin(deg2rad(15)); % amplitude (m)

% Derived parameters
A0 = pi*r0^2*(1-(delta0+1)^4/((1-delta0^2)^2));
fishLength = 4*r0/(1 - delta0);
deltaConstant = (sqrt(pi*r0^2-A0)-sqrt(pi*r0^2))/...
                (sqrt(pi*r0^2-A0)+sqrt(pi*r0^2));
            
% Control parameters
kP = 2;                 % phase control gain
kE = [1,1];             % error dynamics control gain
phaseRateLimit = 5;     % control saturation limit
deadbandLimit = 0.05*fishLength;    % deadband nonlinearity

% Check Strouhaul number is ~0.3
frequency = omega/(2*pi);
amplitude = 2*fishLength*A;
strouhaul = frequency*amplitude/u0;
