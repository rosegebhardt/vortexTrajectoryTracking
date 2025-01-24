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

% Define derivative filter
derivativeFilter = designfilt('differentiatorfir','FilterOrder',50, ...
    'PassbandFrequency',4,'StopbandFrequency',8,'SampleRate',1/deltaT);

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
        end
    end
    
    % Find time index where vortex passes the tail of the fish
    metricIndex = find(r(:,1) > 2*k0,1,'first');
    if isempty(metricIndex)
        error('Reference trajectory integration failure.');
    end
    
    % First-order reference dynamics
    rDot = zeros(timeN,2);
    for ii = 1:timeN
        rDot(ii,:) = vortexConvection(timeSpan(ii),r(ii,:),phi_ref).';
    end
    
    % Store initial condition index
    indexZ = 1;
    
    % Iterate over initial position
    for z_02 = Z_02 
        
        % Update initial position (foil lengths)
        z_0 = [z_01,r_02+z_02]*fishLength;

        % Initialize tracking errors
        e = zeros(timeN,2); e(1,:) = z_0 - r_0; 

        % Initialize control inputs
        phiStar = zeros(timeN,1); phiStar(1) = phi_0;

        % Initialize closed-loop trajectory
        z = zeros(timeN,2); z(1,:) = z_0; 

        % Iterate process over time
        for ii = 1:(timeN-1)

            % Minimize cost function
            costFcn = @(phi) norm(vortexConvection(timeSpan(ii),z(ii,:),phi).' - rDot(ii,:) + kE.*e(ii,:));
            phiStar(ii+1) = fminsearch(costFcn,phiStar(ii));

            % Integrate with chosen phase
            odefunCTRL = @(t,z) vortexConvection(t,z,phiStar(ii+1));
            startTime = tic;
            miniTimeSpan = linspace(timeSpan(ii),timeSpan(ii+1),miniTimeN);
            [~,z_update] = ode23s(odefunCTRL,miniTimeSpan,z(ii,:),...
                                  odeset('Events',@(t,z) timeEvent(t,z,startTime)));
            
            % Reduce intergration tolerance if integration time is exceeded
            if (length(z_update) < miniTimeN)
                startTime = tic;
                [~,z_update] = ode23s(odefunCTRL,miniTimeSpan,z(ii,:),...
                               odeset('RelTol',1e-2,'AbsTol',1e-2,...
                                      'Events',@(t,z) timeEvent(t,z,startTime)));
                if(length(z_update) < miniTimeN)
                    [~,z_update] = ode23s(odefunCTRL,miniTimeSpan,z(ii,:),...
                                   odeset('RelTol',1e-1,'AbsTol',1e-1));
                end
            end

            % Update state and error
            z(ii+1,:) = z_update(end,:);
            e(ii+1,:) = z(ii+1,:) - r(ii+1,:);

        end

        % Account for time delay
        phiStarDotBegin = filter(derivativeFilter,phiStar)/deltaT;
        delay = mean(grpdelay(derivativeFilter));
        phiStarDotBegin(1:delay) = [];

        % Use finite difference to compute derivative at end
        phiStarDotEnd = zeros(delay,1); phiStarEnd = phiStar(end-delay+1:end);
        phiStarDotEnd(1) = (phiStarEnd(2) - phiStarEnd(1))/deltaT;
        for ii = 2:(delay-1)
            phiStarDotEnd(ii) = (phiStarEnd(ii+1) - phiStarEnd(ii-1))/(2*deltaT);
        end
        phiStarDotEnd(end) = (phiStarEnd(end) - phiStarEnd(end-1))/deltaT;

        % Concatinate two derivative arrays
        phiStarDot = [phiStarDotBegin;phiStarDotEnd];

        % Initialize control inputs
        u = zeros(timeN,1);

        % Initialize closed-loop trajectory
        state = zeros(timeN,3);
        state(1,:) = [z_0,phi_0]; 

        % Iterate process over time
        for ii = 1:(timeN-1)
            
            % % Proportional controller without deadband nonlinearity
            % u(ii) = phiStarDot(ii) - kP*sin(state(ii,3) - phiStar(ii));
            
            % Proportional controller with deadband nonlinearity
            if norm(state(ii,1:2) - r(ii,:)) < deadbandLimit
                u(ii) = 0;
            else
                u(ii) = phiStarDot(ii) - kP*sin(state(ii,3) - phiStar(ii));
            end

            % Apply saturation limit
            if u(ii) > phaseRateLimit
                u(ii) = phaseRateLimit;
            end
            if u(ii) < -phaseRateLimit
                u(ii) = -phaseRateLimit;
            end

            % Integrate with chosen control input
            odefunCTRL = @(t,z) stateDynamics(t,z,u(ii));
            startTime = tic;
            [tUpdate,stateUpdate] = ode23s(odefunCTRL,[timeSpan(ii),timeSpan(ii+1)],state(ii,:),...
                                    odeset('Events',@(t,z) timeEvent(t,z,startTime)));
                          
            % Reduce intergration tolerance if integration time is exceeded 
            if(tUpdate(end) ~= timeSpan(ii+1))
                startTime = tic;
                [tUpdate,stateUpdate] = ode23s(odefunCTRL,[timeSpan(ii),timeSpan(ii+1)],...
                                        state(ii,:),odeset('RelTol',1e-2,'AbsTol',1e-2,...
                                        'Events',@(t,z) timeEvent(t,z,startTime)));
                if(tUpdate(end) ~= timeSpan(ii+1))
                    [tUpdate,stateUpdate] = ode23s(odefunCTRL,[timeSpan(ii),timeSpan(ii+1)],state(ii,:),...
                                            odeset('RelTol',1e-1,'AbsTol',1e-1));
                end
            end
                
            % Update state
            state(ii+1,:) = stateUpdate(end,:);
            
        end
        
        % Store success metric
        metricTimeInterval = metricIndex:(metricIndex+round(2*(2*pi/omega)/deltaT));
        ebar = mean(vecnorm(state(metricTimeInterval,1:2) - r(metricTimeInterval,:),2,2));
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
% name conversion: rocPlot_[phi_0 - phi_ref]deg_[z_1(0)]foils.mat
save('rocPlot_180deg_4foils_deadband.mat','errorReduction','R_02','Z_02');

