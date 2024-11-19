%% Problem Initialization: Open-Loop and Reference Trajectories

clearvars; close all; clc;

% Load system parameters
parameters

% ODE45 set up
timeN = 1001; % number of time increments
timeSpan = linspace(0,20,timeN); % time vector (sec)
deltaT = timeSpan(2) - timeSpan(1);

% Open-loop initial conditions
z_0 = [-4,-0.4]*4*k0; % initial position (foil lengths)
phi_0 = 0; 
% z_0 = [-4,0.8]*4*k0; % initial position (foil lengths)
% phi_0 = 0; 

% Reference initial conditions
r_0 = [-4,-0.5]*4*k0; % initial position (foil lengths)
phi_ref = pi/2;
% r_0 = [-4,0.8]*4*k0; % initial position (foil lengths)
% phi_ref = 0;

% Integrate open-loop dynamics
odefunOL = @(t,z) vortexConvection(t,z,phi_0);
[~,zOL] = ode45(odefunOL,timeSpan,z_0);

% Integrate reference dynamics
odefunREF = @(t,z) vortexConvection(t,z,phi_ref);
[~,r] = ode45(odefunREF,timeSpan,r_0);

%% Error Dynamics and Optimal Phase Calculation

% First-order reference dynamics
rDot = zeros(timeN,2);
for ii = 1:timeN
    rDot(ii,:) = vortexConvection(timeSpan(ii),r(ii,:),phi_ref).';
end

% Initialize tracking errors
e = zeros(timeN,2);
e(1,:) = z_0 - r_0; 

% Initialize control inputs
phiStar = zeros(timeN,1);
phiStar(1) = phi_0;

% Initialize closed-loop trajectory
z = zeros(timeN,2);
z(1,:) = z_0; 

% Store cost values for evaluation
minCost = zeros(timeN-1,1);

% Iterate process over time
for ii = 1:(timeN-1)
    
    % Minimize cost function (unconstrained minimization) NO HEURISTIC
    costFcn = @(phi) norm(vortexConvection(timeSpan(ii),z(ii,:),phi).' - rDot(ii,:) + kE.*e(ii,:));
                     % ...+ 0.05*normpdf(phi-phiStar(ii),-omega*deltaT,5e-3);
    phiStar(ii+1) = fminsearch(costFcn,phiStar(ii));
    minCost(ii) = costFcn(phiStar(ii+1));
    
    % Integrate with chosen phase
    odefunCTRL = @(t,z) vortexConvection(t,z,phiStar(ii+1));
    [~,z_update] = ode45(odefunCTRL,[timeSpan(ii),timeSpan(ii+1)],z(ii,:));
    
    % Update state and error
    z(ii+1,:) = z_update(end,:); % TODO + 1e-2*randn(1,2); % + [0,0.05]; (bias makes things weird)
    e(ii+1,:) = z(ii+1,:) - r(ii+1,:);
    
end

%% Phase Rate Control: Backstepping

% Define derivative filter
derivativeFilter = designfilt('differentiatorfir','FilterOrder',50, ...
    'PassbandFrequency',4,'StopbandFrequency',8,'SampleRate',1/deltaT);

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
    
    % Proportional controller with deadband nonlinearity
    if norm(e(ii,:)) < deadbandLimit
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
    [~,stateUpdate] = ode45(odefunCTRL,[timeSpan(ii),timeSpan(ii+1)],state(ii,:));
    
    % Update state
    state(ii+1,:) = stateUpdate(end,:);
    
end

%% Optimal Phase Compuation Results

fig1 = figure(1); 
set(fig1,'units','normalized','outerposition',[0.1 0.1 0.64 1])

% Plot cost function evaluation against time
subplot1_1 = subplot(3,1,1);

plot(timeSpan(1:(timeN-1)),minCost,'-k','Linewidth',2);
ylabel('Cost Evaluation');
ylim([0,0.6]);

ax = gca; ax.GridColor = 'black'; ax.FontSize = 12; grid on; 
ax.YLabel.FontSize = 18; ax.YLabel.Interpreter = 'latex'; 

% Plot reference and desired phase against time
subplot1_2 = subplot(3,1,2);
subplot1_2.Position(2) = 0.44;

yline(phi_ref,'--r','Linewidth',2); hold on;
yline(phi_ref - 2*pi,'--r','Linewidth',2);
yline(phi_ref - 4*pi,'--r','Linewidth',2);
yline(phi_ref - 6*pi,'--r','Linewidth',2);
yline(phi_ref - 8*pi,'--r','Linewidth',2);
plot(timeSpan,phiStar,'-b','Linewidth',2); 
ylabel('Desired Phase (rad)');
legend('Reference Phase ($\pm 2\pi n$)','','','','','Computed Desired Phase')

ax = gca; ax.GridColor = 'black'; ax.FontSize = 12; grid on; box on;
ax.YLabel.FontSize = 18; ax.YLabel.Interpreter = 'latex'; 
ax.Legend.FontSize = 18; ax.Legend.Interpreter = 'latex'; ax.Legend.Location = 'southwest';

% Plot reference and desired camber against time
subplot1_3 = subplot(3,1,3);
subplot1_3.Position(2) = 0.17;

plot(timeSpan,A*cos(omega*timeSpan+phi_ref),'-r','Linewidth',2); hold on;
plot(timeSpan,A*cos(omega*timeSpan+phiStar.'),'-b','Linewidth',2);
xlabel('Time (sec)'); ylabel('Camber Parameter');
legend('Reference Camber','Desired Camber')

ax = gca; ax.GridColor = 'black'; ax.FontSize = 12; grid on; 
ax.XLabel.FontSize = 18; ax.XLabel.Interpreter = 'latex'; 
ax.YLabel.FontSize = 18; ax.YLabel.Interpreter = 'latex'; 
ax.Legend.FontSize = 18; ax.Legend.Interpreter = 'latex'; ax.Legend.Location = 'southwest';

%% Reference Phase Tracking Results

fig3 = figure(3); 
set(fig3,'units','normalized','outerposition',[0.1 0.1 0.64 1])

% Plot phase against time
subplot3_1 = subplot(3,1,1);

plot(timeSpan,phiStar,'--b','Linewidth',2); hold on;
plot(timeSpan,state(:,3),'-k','Linewidth',2)
ylabel('Phase Offset (rad)'); legend('Reference Phase','Closed-Loop Phase');

ax = gca; ax.GridColor = 'black'; ax.FontSize = 12; grid on; 
ax.XLabel.FontSize = 18; ax.YLabel.FontSize = 18; ax.Legend.FontSize = 16;
ax.XLabel.Interpreter = 'latex'; ax.YLabel.Interpreter = 'latex'; ax.Legend.Interpreter = 'latex'; 
ax.Legend.Location = 'southwest'; 

% Plot camber parameter against time
subplot3_2 = subplot(3,1,2);
subplot3_2.Position(2) = 0.44;

plot(timeSpan,A*cos(omega*timeSpan+phiStar.'),'--b','Linewidth',2); hold on;
plot(timeSpan,A*cos(omega*timeSpan+state(:,3).'),'-k','Linewidth',2);
ylabel('Camber Parameter'); legend('Reference Camber','Closed-Loop Camber');

ax = gca; ax.GridColor = 'black'; ax.FontSize = 12; grid on; 
ax.XLabel.FontSize = 18; ax.YLabel.FontSize = 18; ax.Legend.FontSize = 16;
ax.XLabel.Interpreter = 'latex'; ax.YLabel.Interpreter = 'latex'; ax.Legend.Interpreter = 'latex'; 
ax.Legend.Location = 'southwest'; 

% Plot control input against time
subplot3_3 = subplot(3,1,3);
subplot3_3.Position(2) = 0.17;

plot(timeSpan,u,'-k','Linewidth',2); hold on;
yline(-phaseRateLimit,'--k','Linewidth',2);
yline(phaseRateLimit,'--k','Linewidth',2);
ylim([-phaseRateLimit-1,phaseRateLimit+1]);
xlabel('Time (sec)'); ylabel('Control Input (rad/sec)'); 
legend('Control Input','Saturation Limits');

ax = gca; ax.GridColor = 'black'; ax.FontSize = 12; grid on; 
ax.XLabel.FontSize = 18; ax.YLabel.FontSize = 18; ax.Legend.FontSize = 16;
ax.XLabel.Interpreter = 'latex'; ax.YLabel.Interpreter = 'latex'; ax.Legend.Interpreter = 'latex'; 
ax.Legend.Location = 'southwest'; 

%% Trajectory Tracking Results

fig4 = figure(4); 
set(fig4,'units','normalized','outerposition',[0.1 0.1 0.8 0.6])

% Plot phase against time
plot(timeSpan,vecnorm(state(:,1:2)-r,2,2),'-k','Linewidth',2); hold on;
xlabel('Time (sec)'); ylabel('Magnitude of Tracking Error (m)'); 
% legend('Successful Tracking','Unsuccessful Tracking');

ax = gca; ax.GridColor = 'black'; ax.FontSize = 12; grid on; 
ax.XLabel.FontSize = 18; ax.XLabel.Interpreter = 'latex'; 
ax.YLabel.FontSize = 18; ax.YLabel.Interpreter = 'latex'; 
% ax.Legend.FontSize = 16; ax.Legend.Interpreter = 'latex'; ax.Legend.Location = 'southwest'; 

%% Plot Results 

% videoii = 1;
everyNthTime = 10;

for ii = timeN %1:everyNthTime:timeN
    
    fig6 = figure(6); % define figure and size
    set(fig6,'units','normalized','outerposition',[0.1 0.1 1 0.6])
    
    % Camber thickness and transform parameters (open-loop)
    betaOL = A*cos(omega*timeSpan(ii)+phi_0); 
    deltaOL = deltaConstant*sqrt(1-betaOL^2);
    kOL = r0*(deltaOL + sqrt(1 - betaOL^2));
    
    % Show foil profile (open-loop)
    angle = linspace(0,2*pi,1000);
    circleOL = r0*(exp(1i*angle) + deltaOL + 1i*betaOL); 
    foilOL = circleOL + kOL^2./circleOL;
    fill(real(foilOL)/(4*k0),imag(foilOL)/(4*k0),'r'); hold on; % for video

    % Camber thickness and transform parameters
    beta = A*cos(omega*timeSpan(ii)+state(ii,3)); 
    delta = deltaConstant*sqrt(1-beta^2);
    zeta_0 = r0*(delta + 1i*beta);
    k = r0*(delta + sqrt(1 - beta^2)); 
    
    % Show foil profile
    circle = r0*exp(1i*angle) + zeta_0; 
    foil = circle + k^2./circle; 
    fill(real(foil)/(4*k0),imag(foil)/(4*k0),'b'); hold on; % for video
%     fill(real(foil)/(4*k0),imag(foil)/(4*k0),'k'); hold on; % for image
    
    % Show trajectories
    plot(zOL(1:ii,1)/(4*k0),zOL(1:ii,2)/(4*k0),'red','Linewidth',2)
    plot(state(1:ii,1)/(4*k0),state(1:ii,2)/(4*k0),'blue','Linewidth',2)
    plot(r(1:ii,1)/(4*k0),r(1:ii,2)/(4*k0),'color',[0.5 0 0.5],'Linewidth',2)
    
    % Labels and graph parameters
    olLegend = ['Open-Loop Trajectory: $\phi = ',num2str(phi_0),'$'];
    deLegend = ['Desired Trajectory: $\phi = ',num2str(phi_ref),'$'];
    legend('Open-Loop Fish','Closed-Loop Fish','Open-Loop Trajectory',...
           'Closed-Loop Trajectory','Desired Trajectory'); % for video
       
%     legend('Fish Profile','Open-Loop Trajectory',...
%            'Closed-Loop Trajectory','Desired Trajectory') % for image
    axis equal; xlim([-4,2]); ylim([-1,1]);
    
    xlabel('$z_1$ (Foil Lengths)','interpreter','latex'); 
    ylabel('$z_2$ (Foil Lengths)','interpreter','latex');
    ax = gca; ax.FontSize = 12;
    ax.XLabel.FontSize = 28; ax.YLabel.FontSize = 28;
    ax.Legend.FontSize = 20; ax.Legend.Interpreter = 'latex'; ax.Legend.Location = 'northeast';
    hold off;
    
%     % Store plot
%     video(videoii) = getframe(gcf);
%     drawnow
%     videoii = videoii + 1;

end

% % Create video from plots
% writerObj = VideoWriter('phaseControlVideo.avi');
% writerObj.FrameRate = 50;
% 
% % Convert to frame
% open(writerObj)
% for index = 1:length(video)
%     frame = video(index);
%     writeVideo(writerObj,frame);
% end
% 
% close(writerObj);
