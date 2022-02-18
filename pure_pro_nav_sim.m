%--------------------------------------------------------------------------
% Pure Proportional Navigation Guidance Law 
%
% * This script contains an over-simplified kinematics simulation that
%   demonstrates the use of pure pro-nav for target pursuit. 
%
% * All positions, velocities, and angels are in the inertial frame (NED).
%
% * Kinematics assume all commands are executed perfectly and immediately.
% 
% Resources:
% * Zarchan, P., 2007. Tactical and Stratigic Missile Guidance, 
%   AIAA Press, 
% 
% * Stevens, B.L., Lewis, F.L. and Johnson, E.N., 2015. Aircraft control 
%   and simulation: dynamics, controls design, and autonomous systems.
%   John Wiley & Sons.
% 
% * Ben Dickinson's toturial series: 
%   - Kinematics for Simulation of Proportional Navigation Rev 2
%     Section 3 Module 2 - Missile Guidance
%   - How to Code Proportional Navigation in Matlab or Octave
%     Missile Guidance Fundamentals - Appendix A
%
% Ben Raanan, Monterey Bay Aquarium Research Institute (MBARI), Feb 12 2022
%--------------------------------------------------------------------------
clear; close all

% Pro-Nav gain, unitless (typically set between 3-5)
N = 4;

% Pursuer init prams
pursuer.N   =  0; % north pos, m
pursuer.E   =  0; % east pos, m
pursuer.psi =  0; % heading, rad
pursuer.V   =  1.0; % velocity magnitude, m/s
pursuer.Nv  =  pursuer.V * cos(pursuer.psi); % north velocity, m/s
pursuer.Ev  =  pursuer.V * sin(pursuer.psi); % east velocity, m/s

% Target init prams
target.anchored = false; % fix target to init pos when true
target.N   =  250; % north pos, m
target.E   =  250;   % east pos, m
target.Nv  =  0.2; % north velocity, m/s
target.Ev  =  0.5; % east velocity, m/s
target.V   = sqrt(target.Nv^2 + target.Ev^2); % velocity magnitude, m/s

% Current prams
% Applied as a disturbance to pursuer's and target's velocity
current.Nv =  0.0; % north velocity, m/s
current.Ev =  0.0; % east velocity, m/s

% Sim params
S  = 1250;    % max sim duration, seconds
dt = 0.4;     % time-step size, seconds
Niter = S/dt; % max iteration num

% Pre-allocate logger
logger.t   = nan(1, Niter);    % elapsed time
logger.pN  = nan(1, Niter);    % pursuer north pos
logger.pE  = nan(1, Niter);    % pursuer east pos
logger.pNv = nan(1, Niter);    % pursuer north vel
logger.pEv = nan(1, Niter);    % pursuer east vel
logger.psi = nan(1, Niter);    % pursuer heading angle

logger.tN  = nan(1, Niter);    % target north pos
logger.tE  = nan(1, Niter);    % target east pos
logger.tNv = nan(1, Niter);    % target north vel
logger.tEv = nan(1, Niter);    % target east vel

logger.R   = nan(1, Niter);    % pursuer/target range
logger.Lambda = nan(1, Niter); % pursuer/target bearing

%--------------------------------------------------------------------------
% Init sim
%--------------------------------------------------------------------------
RTP_last = [(target.N - pursuer.N);... % delta N
    (target.E - pursuer.E)];	       % delta E

% Target pos
target.N = target.N + target.Nv*dt;
target.E = target.E + target.Ev*dt;

% Pursuer pos
pursuer.N = pursuer.N + pursuer.Nv*dt;
pursuer.E = pursuer.E + pursuer.Ev*dt;

%--------------------------------------------------------------------------
% Run sim
%--------------------------------------------------------------------------
for k = 1:Niter
    
    % Relative position in the inertial frame, m
    RTP = [(target.N - pursuer.N);... % delta N
        (target.E - pursuer.E)];      % delta E
    
    % Range to target
    R = norm(RTP);
    
    % Relative velocity in the inertial frame, m/s
    % VTP = [(target.Nv - pursuer.Nv);... % delta N velocity
    %        (target.Ev - pursuer.Ev)];   % delta E velocity
    VTP = (RTP - RTP_last) ./ dt;
    
    % Closing velocity, m/s
    % Vc = -RTP'*VTP / R;
    
    % Pursuer velocity, m/s
    Vp = sqrt(pursuer.Nv^2 + pursuer.Ev^2);
    
    % Target velocity, m/s
    % Vt = sqrt(target.Nv^2 + target.Ev^2);
    
    % Line-of-sight (LOS) angle, rad
    lambda = atan2(RTP(2), RTP(1));
    
    % LOS angle time derivative (d/dt lambda), rad/s
    lambda_dot = (RTP(1)*VTP(2) - RTP(2)*VTP(1)) / R^2;
    
    % Target heading, rad
    % beta = atan2(target.Ev, target.Nv);
    
    % Lead angle, rad
    % L = asin(Vt * sin(beta + lambda) / Vp);
    
    % True Proportional Navigation, rad/s2
    % nc = N * Vc * lambda_dot;
    
    %-------------------------------------
    % Pure Proportional Navigation
    ap = N * Vp * lambda_dot;
    %-------------------------------------
    
    % Terminate sim at intercept
    if abs(R) <= 0.25
        break;
    end
    
    % Update pursuer pos for time-step and apply current slip
    pursuer.N = pursuer.N + pursuer.Nv*dt + current.Nv*dt;
    pursuer.E = pursuer.E + pursuer.Ev*dt + current.Ev*dt;
    
    % Compute the N/E acceleration commands
    % In pure pro-nav accel commands are applied normal
    % to pursuer's velocity vector
    pNa = -ap * sin(pursuer.psi);
    pEa =  ap * cos(pursuer.psi);
    
    % Update pursuer N/E velocities
    pursuer.Nv = pursuer.Nv + pNa*dt;
    pursuer.Ev = pursuer.Ev + pEa*dt;
    
    % Update pursuer heading
    pursuer.psi = atan2(pursuer.Ev, pursuer.Nv);
    
    % Update target pos for time step
    if(~target.anchored)
        target.N = target.N + target.Nv*dt + current.Nv*dt;
        target.E = target.E + target.Ev*dt + current.Ev*dt;
    end
    RTP_last = RTP;
    
    %-------------------------------------
    % Log time-step data
    logger.t(k)   = k*dt;
    logger.pN(k)  = pursuer.N;
    logger.pE(k)  = pursuer.E;
    logger.pNv(k) = pursuer.Nv;
    logger.pEv(k) = pursuer.Ev;
    logger.psi(k) = pursuer.psi;
    logger.tN(k)  = target.N;
    logger.tE(k)  = target.E;
    logger.tNv(k) = target.Nv;
    logger.tEv(k) = target.Ev;
    logger.R(k)   = R;
    logger.Lambda(k) = lambda;
    
end

%--------------------------------------------------------------------------
% Visualize results
%--------------------------------------------------------------------------
close all;

% Impact index
[M,I] = min(logger.R);

% Range
%-------------------------------------
figure;
plot(logger.t,logger.R); hold on;
scatter(I*dt, M, 'filled')
title(['Pure Proportional Navigation, N = ' num2str(N) ])
legend('Range',...
    ['Intercept: r = ' num2str(M) ' m, t = ' num2str(I*dt) ' s'],...
    'Location','nw')
ylabel('Range (m)')
xlabel('Elapsed time (sec)')
set(gca, 'YDir', 'reverse')

% Heading
%-------------------------------------
rad2deg = @(x) x*180/pi;  % helper function

figure;
plot(logger.t, rad2deg(logger.psi)); hold on;
plot(logger.t, rad2deg(logger.Lambda)); hold on;
title(['Pure Proportional Navigation, N = ' num2str(N) ])
legend('Pursuer heading', 'Bearing to target', 'Location', 'nw' )
ylabel('Heading angle (deg)')
xlabel('Elapsed time (sec)')

% Position
%-------------------------------------
figure;
scatter(logger.pE, logger.pN, 'filled'); hold on;
scatter(logger.tE, logger.tN, 'filled');
set(gca, 'DataAspectRatio',[1 1 1]);
title(['Pure Proportional Navigation, N = ' num2str(N) ])
legend('Pursuer', 'Target', 'Location', 'southoutside',...
    'Orientation','horizontal')
xlabel('+E (m)')
ylabel('+N (m)')
