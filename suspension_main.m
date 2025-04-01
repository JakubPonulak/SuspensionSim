% user input for height
heightBump = input("Enter bumb height in meters (range from 0 - 0.25) \n");
wheelRadius = 0.35;
% width and time duration of bump
widthBump = heightBump * 20;
timeBump = heightBump*2; 
% velocity = bump width / time to cover bump = 10 m/s

function dxdt = suspension_ode(t, x, M_s, M_u, k_s, k_t, c_s, h, T)
    % differential equation variables
    % setting them equal to components of state vector x
    y_c = x(1); % displacement of the car body
    y_c_dot = x(2); % rate of change of y_c
    y_w = x(3); % displacement of the wheel
    y_w_dot = x(4); % rate of change of y_w
    
    % equation for bump shape
    if t <= T
        y_r = h * sin(pi * t / T)^2; % Smooth bump shape
    else
        y_r = 0; % make road flat after bump
    end

    % Equations of motion
    dy_c_dot = (-c_s * (y_c_dot - y_w_dot) - k_s * (y_c - y_w)) / M_s;
    dy_w_dot = (c_s * (y_c_dot - y_w_dot) + k_s * (y_c - y_w) - k_t * (y_w - y_r)) / M_u;

    % Return derivatives as a column vector
    dxdt = [y_c_dot; dy_c_dot; y_w_dot; dy_w_dot];
end

% Define system parameters
M_s = 500;    % Sprung mass (kg)
M_u = 50;     % Unsprung mass (kg)
k_s = 20000;  % Suspension stiffness (N/m)
k_t = 200000; % Tire stiffness (N/m) (potentially add slider)
c_s = 7000;   % Damping coefficient (Ns/m)

% Define simulation time (not real seconds)
tspan = [0 1];
x0 = [0; 0; 0; 0]; % Initial conditions [y_c, y_c_dot, y_w, y_w_dot]

% Solve the system using ode78
[t, X] = ode78(@(t, x) suspension_ode(t, x, M_s, M_u, k_s, k_t, c_s, heightBump, timeBump), tspan, x0);

% Extract results from solution array X
y_c = X(:,1); % Car body displacement
y_w = X(:,3); % Wheel displacement
idx1 = 1:3:length(y_w); % extract every 3rd element of y_w to increase sim speed
y_w = y_w(idx1);
idx2 = 1:3:length(y_c); % extract every 3rd element of y_c to increase sim speed
y_c = y_c(idx2);

% function for car wheel and body graphic
function carGraphic(r1, r2, y1, y2)
    % drawing the wheel
    th1=0:0.05:2*pi; 
    % drawing 3 concentric circles of varying radii
    xTire = r1 * cos(th1);
    xWheel = (r1*0.75) * cos(th1); 
    xInner = (r1*0.25) * cos(th1);
    yTire = r1 * sin(th1);
    yWheel = (r1*0.75) * sin(th1);
    yInner = (r1*0.25) * sin(th1);
    
    % drawing the car body
    th2=0:0.01:pi; 
    xRim = r2 * cos(th2);
    yRim = r2 * sin(th2);
    xFill = [xRim, fliplr(xRim)];
    yFill = [5 * ones(size(xRim)), fliplr(yRim+y2+r2)];

    % colormaps for custom colors for the wheel hubcap
    colorMap1 = [0.69 0.69 0.69]; % RGB triplet for light grey color
    colorMap2 = [0.5 0.5 0.5]; % RGB triplet for dark grey color

    % plotting circles and filling in areas with colors
    plot(xTire,yTire+y1+r1-0.01, 'black')
    fill(xTire,yTire+y1+r1-0.01, 'black')
    hold on
    plot(xWheel,yWheel+y1+r1-0.01)
    fill(xWheel,yWheel+y1+r1-0.01, colorMap1)
    plot(xInner,yInner+y1+r1-0.01)
    fill(xInner,yInner+y1+r1-0.01, colorMap2)
    
    % plotting the car body
    plot(xRim,yRim+y2+r2, 'black', 'LineWidth', 2)
    plot([-3, -r2], [y2+r2, y2+r2], 'black', 'LineWidth', 2)
    plot([r2, 3], [y2+r2, y2+r2], 'black', 'LineWidth', 2)
    fill(xFill, yFill, 'r', 'EdgeColor', 'none')
    fill([-r2 -r2 -3 -3], [y2+r2, y2+r2+3, y2+r2+3, y2+r2], 'r', 'EdgeColor', 'none')
    fill([r2 r2 3 3], [y2+r2, y2+r2+3, y2+r2+3, y2+r2], 'r', 'EdgeColor', 'none')
    
    hold off
end

% function for road bump graphic
function bumpGraph(w,time)
    t = time:0.01:(w+time);
    y_r = (w/20) * sin(pi * (t - time)/ w).^2; % Smooth bump shape
    plot(t, y_r, 'b')
    hold on
    colorMap = [0.4 0.4 0.4]; % custom color for road
    yline(0, 'Color', [0.57 0.57 0.57]) % different color than road, but looks the same in figure
    fill([t -t], [y_r zeros(size(y_r))], colorMap)
    fill([-1 2 2 -1], [0 0 -0.1 -0.1], colorMap)
    hold off
end

% animation loop
for u = 1:length(y_w)
    %function to draw wheel
    carGraphic(wheelRadius, wheelRadius * 1.3, y_w(u), y_c(u))
    hold on
    %function to draw road and bump
    bumpGraph(widthBump, (u*-0.125 + 1))
    % u*-0.125 + 1 is adjustment to match bump speed with wheel
    hold off
    xlim([-1 1])
    ylim([-0.1 1.5])
    pbaspect([1.4 1 1]) % square aspect ratio
    pause(0.01)
end

