%% ARCHIVED: USE REDUCED REGION OF CONVERGENCE!!!

%% Problem Initialization

clearvars; close all; clc;

% Load system parameters
parameters

% ODE45 set up
timeN = 1001; miniTimeN = 10;
timeSpan = linspace(0,20,timeN); % time vector (sec)
deltaT = timeSpan(2) - timeSpan(1);

% Fixed open-loop and reference phases
z1InitPosition = -4;
phi_ref = 0; phi_0 = 0; 
odefunREF = @(t,z) vortexConvection(t,z,phi_ref);

% Control gains and saturation limit
kE = [1,1]; kP = 2;
phaseRateLimit = 5;

% Define derivative filter
derivativeFilter = designfilt('differentiatorfir','FilterOrder',50, ...
    'PassbandFrequency',4,'StopbandFrequency',8,'SampleRate',1/deltaT);

%% TODOs

% 1. Change initial phase offset (increments of 90 degrees)
% 2. Change initial lateral position (increments of 1 foil length)

% Add results for failure cases?

%% Loop Through Initial Conditions

% Open-loop and reference initial conditions and error meshgrid
nZ = 21; Z_02 = linspace(-1,1,nZ);
nR = 21; R_02 = linspace(-1,1,nR);
errorReduction = zeros(nZ,nR);

% Store reference trajectory index
indexR = 1;

% Iterate over reference trajectory
for r_02 = R_02
     
    % Update reference trajectory
    r_0 = [z1InitPosition,r_02]*4*k0;
    
    % Integrate reference dynamics
    startTime = tic;
    [~,r] = ode45(odefunREF,timeSpan,r_0,...
            odeset('Events',@(t,z) timeEventLong(t,z,startTime,k0)));
        
    % Update state and terminate loop if integration fails 
    if(length(r) < timeN)
        
        % Rename length(r) = timeN, do everything until 
        
        % Display statement
        dispStatement = ['nR = ',num2str(indexR),': Integration Failed'];
        disp(dispStatement);
            
        % Loop updates
        errorReduction(:,indexR) = NaN;
        indexR = indexR + 1; continue
        
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
        
        % Update initial position
        z_0 = [z1InitPosition,z_02]*4*k0;

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
            [~,z_update] = ode45(odefunCTRL,miniTimeSpan,z(ii,:),...
                                 odeset('Events',@(t,z) timeEvent(t,z,startTime,k0)));
            
            % End process if integration fails
            if (length(z_update) < miniTimeN)
                break
            end

            % Update state and error
            z(ii+1,:) = z_update(end,:);
            e(ii+1,:) = z(ii+1,:) - r(ii+1,:);

        end
        
        % Update state and terminate loop if integration fails
        if (length(z_update) < miniTimeN)
            
            % Display statement
            dispStatement = ['nR = ',num2str(indexR),', nZ = ',num2str(indexZ),...
                             ': Integration Failed at Optimization Step.'];
            disp(dispStatement);
            
            % Loop updates
            errorReduction(indexZ,indexR) = NaN; 
            indexZ = indexZ + 1; continue
            
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

            % Proportional controller
            u(ii) = phiStarDot(ii) - kP*sin(state(ii,3) - phiStar(ii));

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
            [tUpdate,stateUpdate] = ode45(odefunCTRL,[timeSpan(ii),timeSpan(ii+1)],state(ii,:),...
                                    odeset('Events',@(t,z) timeEvent(t,z,startTime,k0)));
                          
            % Terminate if integration fails
            if(tUpdate(end) ~= timeSpan(ii+1))
                break
            end
                
            % Update state
            state(ii+1,:) = stateUpdate(end,:);
            
        end
        
        % Update state and terminate loop if integration fails
        if(tUpdate(end) ~= timeSpan(ii+1))
            
            % Display statement
            dispStatement = ['nR = ',num2str(indexR),', nZ = ',num2str(indexZ),...
                             ': Integration Failed at Feedback Step.'];
            disp(dispStatement);
            
            % Loop updates
            errorReduction(indexZ,indexR) = NaN; 
            indexZ = indexZ + 1; continue
            
        end
                
        % % Store success metric (archive)
        % e0 = norm(state(1,1:2) - r(1,:));
        % ebar = mean(vecnorm(state(751:end,1:2) - r(751:end,:),2,2));
        % if e0 == 0
        %     errorReduction(indexZ,indexR) = NaN;
        % else
        %     errorReduction(indexZ,indexR) = 1 - ebar/e0;
        % end
        
        % Store success metric
        ebar = mean(vecnorm(state(751:end,1:2) - r(751:end,:),2,2));
        errorReduction(indexZ,indexR) = ebar/(4*k0);
        
        % Display statement
        dispStatement = ['nR = ',num2str(indexR),', nZ = ',num2str(indexZ),...
                         ': Integration Successful!'];
        disp(dispStatement);
        
        % Update reference trajectory index
        indexZ = indexZ + 1;
        
    end
    
    % Update initial condition index
    indexR = indexR + 1;
    
end

% Store for later
% name conversion: rocPlot_[phase offset]deg_[z_1(0)]foils.mat
save('rocPlot_000deg_4foils.mat','errorReduction','R_02','Z_02');
