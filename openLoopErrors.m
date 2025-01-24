%% Problem Initialization: Open-Loop and Reference Trajectories

clearvars; close all; clc;

% Load system parameters
parameters

% ODE23S set up
timeN = 1001;   % number of time increments
miniTimeN = 10; % time increments for short integration periods
timeSpan = linspace(0,20,timeN); % time vector (sec)
deltaT = timeSpan(2) - timeSpan(1);

% Reference and true initial conditions
z_01 = -4;      % foil lengths
phi_ref = 0;    % reference phase offset
phi_0 = pi;      % true phase offset
odefunREF = @(t,z) vortexConvection(t,z,phi_ref);
odefunOL = @(t,z) vortexConvection(t,z,phi_0);

%% Loop Through Initial Conditions

% Open-loop and reference initial conditions and error meshgrid
nZ = 21; Z_02 = linspace(-0.2,0.2,nZ);
nR = 21; R_02 = linspace(-1,1,nR);
errorReduction = zeros(nZ,nR);

% Store reference trajectory index
indexR = 1;

% Iterate over reference trajectory
for r_02 = R_02
    
    % Update reference trajectory (foil lengths)
    r_0 = [z_01,r_02]*fishLength;
    
    % Integrate reference dynamics
    startTime = tic;
    [~,r] = ode23s(odefunREF,timeSpan,r_0,...
            odeset('Events',@(t,z) timeEventLong(t,z,startTime)));
    
    % Reduce intergration tolerance if integration time is exceeded 
    if length(r) < timeN
        startTime = tic;
        [~,r] = ode23s(odefunREF,timeSpan,r_0,odeset('RelTol',1e-2,...
                'AbsTol',1e-2,'Events',@(t,z) timeEventLong(t,z,startTime)));
        if length(r) < timeN
            [~,r] = ode23s(odefunREF,timeSpan,r_0,odeset('RelTol',1e-1,'AbsTol',1e-1));
%             startTime = tic;
%             [~,r] = ode23s(odefunREF,timeSpan,r_0,odeset('RelTol',1e-1,...
%                     'AbsTol',1e-1,'Events',@(t,z) timeEventLong(t,z,startTime)));
%             if length(r) < timeN
%                 [~,r] = ode23s(odefunREF,timeSpan,r_0,odeset('RelTol',1e0,'AbsTol',1e0));
%             end
        end
    end
    
    % Find time index where vortex passes the tail of the fish
    metricIndex = find(r(:,1) > 2*k0,1,'first');
    if isempty(metricIndex)
        error('Reference trajectory integration failure.');
    end
        
    % Store initial condition index
    indexZ = 1;
    
    % Iterate over initial position
    for z_02 = Z_02 
        
        % Update initial position (foil lengths)
        z_0 = [z_01,r_02+z_02]*fishLength;
        
        % Integrate reference dynamics
        startTime = tic;
        [~,z] = ode23s(odefunOL,timeSpan,z_0,...
                odeset('Events',@(t,z) timeEventLong(t,z,startTime)));
        
        % Reduce intergration tolerance if integration time is exceeded 
        if length(z) < timeN
            startTime = tic;
            [~,z] = ode23s(odefunOL,timeSpan,z_0,odeset('RelTol',1e-2,...
                    'AbsTol',1e-2,'Events',@(t,z) timeEventLong(t,z,startTime)));
            if length(z) < timeN
                [~,z] = ode23s(odefunOL,timeSpan,z_0,odeset('RelTol',1e-1,'AbsTol',1e-1));
%                 startTime = tic;
%                 [~,z] = ode23s(odefunOL,timeSpan,z_0,odeset('RelTol',1e-1,...
%                         'AbsTol',1e-1,'Events',@(t,z) timeEventLong(t,z,startTime)));
%                 if length(z) < timeN
%                     [~,z] = ode23s(odefunOL,timeSpan,z_0,odeset('RelTol',1e0,'AbsTol',1e0));
%                 end
            end
        end
        
        % Store success metric
        metricTimeInterval = metricIndex:(metricIndex+round(2*(2*pi/omega)/deltaT));
        ebar = mean(vecnorm(z(metricTimeInterval,:) - r(metricTimeInterval,:),2,2));
        errorReduction(indexZ,indexR) = ebar/fishLength;
        
        % Display statement
        dispStatement = ['nR = ',num2str(indexR),', nZ = ',num2str(indexZ)];
        disp(dispStatement);
        
        % Update reference trajectory index
        indexZ = indexZ + 1;
        
    end
    
    % Update initial condition index
    indexR = indexR + 1;
    
end

%% Store data

% Store for later
% name conversion: rocPlot_[phi_0 - phi_ref]deg_[z_1(0)]foils_openLoop.mat
save('rocPlot_180deg_4foils_openLoop.mat','errorReduction','R_02','Z_02');

