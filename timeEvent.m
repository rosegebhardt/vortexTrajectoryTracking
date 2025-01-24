function [values,isterminal,direction] = timeEvent(t,z,startTime)

% This function terminates integration if it takes too long. This prevents
% MATLAB from freezing if integration hits a singular or near-singular
% condition. 

    values(1) = toc(startTime) < 1; % only allow one second of runtime
    isterminal(1) = 1;
    direction(1) = 0;
    
    % values(2) = z(1) < 2*fishLength; % vortex two fish lengths behind tail
    % isterminal(2) = 0;
    % direction(2) = 0;
    
end