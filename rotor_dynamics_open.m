%% Rotor dynamics - Open loop torque input
% Simulation and animation of a rotor with open loop torque input.
%
%%

clear ; close all ; clc

%% Circle
th_c    = 0:0.01:2*pi;          % angle for sweep               [rad]
r       = 1;                    % Radius                        [m]
x_c     = r*cos(th_c);          % Position x                    [m]
y_c     = r*sin(th_c);          % Position y                    [m]

%% Simulation
tf      = 60;                   % Final time                    [s]
fR      = 30;                   % Frame rate                    [fps]
dt      = 1/fR;                 % Time resolution               [s]
time    = linspace(0,tf,tf*fR); % Time                          [s]

% Initial conditions [position speed]
th_0    = 0;                    % Initial angular position      [rad]
w_0     = 0;                    % Initial angular speed         [rad/s]
z0      = [0 0];

options = odeset('RelTol',1e-6);
[T, Z]  = ode45(@rotor_dynamics, time, z0,options);

%% Retrieving states
th      = Z(:,1);               % angular position              [rad]
w       = Z(:,2);               % angular speed                 [rad/s]
% Preallocating
torque  = zeros(1,length(T));
for i=1:length(T)
  [dz, torque(i)] = rotor_dynamics(T(i),Z(i,:));
end

% Rotation indicator
x = r*cos(th);                  % Coordinate x                  [m]
y = r*sin(th);                  % Coordinate y                  [m]

%% Animation

figure
set(gcf,'Position',[270   140   640     360  ])

% Create and open video writer object
v = VideoWriter('rotor_dynamics_open.avi');
v.Quality = 100;
open(v);

for i=1:length(time)
    subplot(2,3,[1 2 4 5])
      cla
      hold on ; grid on ; axis equal ; box on
      set(gca,'xlim',[-1.2*r 1.2*r],'ylim',[-1.2*r 1.2*r])
      set(gca,'XTick',[], 'YTick', [])
      plot(x_c,y_c,'k','linewidth',3)               % Circle
      plot([-x(i) x(i)],[-y(i) y(i)],'r--')         % Rotation indicator
      plot(0,0,'k*')                                % Origin
      title('Rotor')
    subplot(2,3,3)
      cla
      hold on ; grid on ; box on
      set(gca,'xlim',[T(1) T(end)],'ylim',[min(torque)-0.1*max(torque) max(torque)+0.1*max(torque)])
      plot(T,torque,'b')
      plot([T(i) T(i)],[min(torque)-0.1*max(torque) max(torque)+0.1*max(torque)],'k--')
      xlabel('Time [s]')
      ylabel('Torque [N.m]')
    subplot(2,3,6)
      cla
      hold on ; grid on ; box on
      set(gca,'xlim',[T(1) T(end)],'ylim',[min(w)-0.1*max(w) max(w)+0.1*max(w)])
      plot(T,w,'b')
      plot([T(i) T(i)],[min(w)-0.1*max(w) max(w)+0.1*max(w)],'k--')
      xlabel('Time [s]')
      ylabel('Angular speed [rad/s]')
  
    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);

%% Auxiliary functions

function [dz, torque] = rotor_dynamics(t,z)

    % Parameters
    I = 2;                      % Moment of inertia             [kg.m2]
    c = 1;                      % Viscous friction constant     [N.m.s/rad]
    
%     th = z(1);                  % Angular position              [rad]
    w = z(2);                   % Angular speed                 [rad/s]

    % Torque [N.m]
    if t < 2
        torque = 0;
    elseif t < 20
        torque = 8;
    elseif t < 40
        torque = 4; 
    else
        torque = 8;
    end

    % Rotor dynamics
    dz(1,1) = w;
    dz(2,1) = (torque - c*w)/I;
  
end