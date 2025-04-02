classdef suspension_sim_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                     matlab.ui.Figure
        RunSimulationButton          matlab.ui.control.Button
        TireStiffnesskNmSlider       matlab.ui.control.RangeSlider
        TireStiffnesskNmSliderLabel  matlab.ui.control.Label
        TireRadiusmSlider            matlab.ui.control.RangeSlider
        TireRadiusmSliderLabel       matlab.ui.control.Label
        BumpHeightmSlider            matlab.ui.control.RangeSlider
        BumpHeightmSliderLabel       matlab.ui.control.Label
        UIAxes2                      matlab.ui.control.UIAxes
        UIAxes                       matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        tireRadius % radius of the wheel in m
        bumpHeight % height of the bump in m
        tireStiffness % stiffness of the tire in kN/m
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: RunSimulationButton
        function RunSimulationButtonPushed(app, event)
            % Disable the button while the simulation runs
            app.RunSimulationButton.Enable = 'off';
        
            % set the width of the bump and the time to traverse it 
            % based on the bump height
            widthBump = app.bumpHeight * 20;
            timeBump = app.bumpHeight * 2;

            % create function to set up problem for ode78
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

            % function for car wheel and body graphic
            function carGraphic(r1, r2, y1, y2)
                
                % drawing the car wheel
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
                xFill = [xRim, fliplr(xRim)]; % area in x to use in fill function
                yFill = [5 * ones(size(xRim)), fliplr(yRim+y2+r2)]; % area in y to use in fill function
            
                % colormaps for custom colors for the wheel hubcap
                colorMap1 = [0.69 0.69 0.69]; % RGB triplet for light grey color
                colorMap2 = [0.5 0.5 0.5]; % RGB triplet for dark grey color
            
                 % plotting connector between wheel and body
                fill(app.UIAxes, [-r1*0.15 -r1*0.3 r1*0.3 r1*0.15 ], [r1+y1, 2+y1, 2+y1, r1+y1], colorMap2)
                hold(app.UIAxes, 'on')
                
                % plotting circles and filling in areas with colors
                plot(app.UIAxes, xTire,yTire+y1+r1-0.01, 'black')
                fill(app.UIAxes, xTire,yTire+y1+r1-0.01, 'black')
                plot(app.UIAxes, xWheel,yWheel+y1+r1-0.01)
                fill(app.UIAxes, xWheel,yWheel+y1+r1-0.01, colorMap1)
                plot(app.UIAxes, xInner,yInner+y1+r1-0.01)
                fill(app.UIAxes, xInner,yInner+y1+r1-0.01, colorMap2)

                % plotting the car body
                plot(app.UIAxes, xRim,yRim+y2+r2, 'black', 'LineWidth', 2)
                fill(app.UIAxes, xFill, yFill, 'r', 'EdgeColor', 'none')
                plot(app.UIAxes, [-3, -r2], [y2+r2, y2+r2], 'black', 'LineWidth', 2)
                fill(app.UIAxes, [-3 -3 -r2 -r2 ], [y2+r2, y2+r2+3, y2+r2+3, y2+r2], 'r', 'EdgeColor', 'none')
                plot(app.UIAxes, [r2, 3], [y2+r2, y2+r2], 'black', 'LineWidth', 2)
                fill(app.UIAxes, [r2 r2 3 3], [y2+r2, y2+r2+3, y2+r2+3, y2+r2], 'r', 'EdgeColor', 'none')
                
                hold(app.UIAxes, 'off')
            end
            
            % function for road bump graphic
            function bumpGraph(w,time)
                t = time:0.01:(w+time);
                y_r = (w/20) * sin(pi * (t - time)/ w).^2; % Smooth bump shape
                plot(app.UIAxes, t, y_r, 'black')
                hold(app.UIAxes, 'on');
                colorMap = [0.4 0.4 0.4]; % custom color for road
                yline(app.UIAxes, 0, 'Color', colorMap)
                fill(app.UIAxes, [t -t], [y_r zeros(size(y_r))], colorMap, 'EdgeColor', 'none')
                fill(app.UIAxes, [-1 2 2 -1], [0 0 -0.1 -0.1], colorMap, 'EdgeColor', 'none')
                hold(app.UIAxes, 'off');
                xlim(app.UIAxes, [-1 1])
                ylim(app.UIAxes, [-0.1 1.5])
            end
            
            %function to run animation of car body and bump
            function runAnimation(widthBump, timeBump)
                
                % Define system parameters for ode78
                M_s = 500;    % Sprung mass (kg)
                M_u = 50;     % Unsprung mass (kg)
                k_s = 20000;  % Suspension stiffness (N/m)
                c_s = 7000;   % Damping coefficient (Ns/m)
                
                % Define simulation time (not real seconds)
                tspan = [0 1];
                x0 = [0; 0; 0; 0]; % Initial conditions [y_c, y_c_dot, y_w, y_w_dot]
                
                % Solve the system using ode78
                [t, X] = ode78(@(t, x) suspension_ode(t, x, M_s, M_u, k_s, app.tireStiffness, c_s, app.bumpHeight, timeBump), tspan, x0);
                
                % Extract results from solution array X
                y_c = X(:,1); % Car body displacement
                y_w = X(:,3); % Wheel displacement
                
                % shortening results for animation to make it run faster
                idx1 = 1:3:length(y_w); % extract every 3rd element of y_w to increase sim speed
                y_w_new = y_w(idx1);
                idx2 = 1:3:length(y_c); % extract every 3rd element of y_c to increase sim speed
                y_c_new = y_c(idx2);
               
                % animation for loop
                for u = 1:length(y_w_new)
                    % Draw wheel
                    carGraphic(app.tireRadius, app.tireRadius * 1.15, y_w_new(u), y_c_new(u))
                    hold(app.UIAxes, 'on');
                    
                    % Draw road and bump
                    bumpGraph(widthBump, (u*-0.125 + 1)); % Adjust bump speed to match wheel
                    hold(app.UIAxes, 'off');
                    
                    % update the plot in real-time
                    drawnow;
                    pause(0.01); % Control animation speed
                end

                % plot graphical results on second figure
                plot(app.UIAxes2, t, y_c, 'r', 'LineWidth', 1.5); 
                hold(app.UIAxes2, 'on');
                plot(app.UIAxes2, t, y_w, 'b', 'LineWidth', 1.5);
                legend(app.UIAxes2,'Car Body', 'Wheel');
                grid(app.UIAxes2, 'on');
                hold(app.UIAxes2, 'off');
            end
            
            % Re-enable the button after the simulation completes
            runAnimation(widthBump, timeBump)

            app.RunSimulationButton.Enable = 'on';
        end

        % Value changed function: TireStiffnesskNmSlider
        function TireStiffnesskNmSliderValueChanged(app, event)
            value = app.TireStiffnesskNmSlider.Value;
            app.tireStiffness = value(2) * 10^3; % choose second slider value as value for function
            % slider is is kN/m, but ode takes N/m, so multiply slider value
            % by 1000, or 10^3
        end

        % Value changed function: TireRadiusmSlider
        function TireRadiusmSliderValueChanged(app, event)
            value = app.TireRadiusmSlider.Value;
            app.tireRadius = value(2); % choose second slider value as value for function
        end

        % Value changed function: BumpHeightmSlider
        function BumpHeightmSliderValueChanged(app, event)
            value = app.BumpHeightmSlider.Value;
            app.bumpHeight = value(2); % choose second slider value as value for function
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1180 673];
            app.UIFigure.Name = 'MATLAB App';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'Car Suspension Simulation')
            ylabel(app.UIAxes, 'Height (m)')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.LabelFontSizeMultiplier = 1.3;
            app.UIAxes.YLim = [-0.1 1.5];
            app.UIAxes.XTick = [];
            app.UIAxes.XTickLabel = '';
            app.UIAxes.YTick = [-0.1 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1 1.1 1.2 1.3 1.4 1.5];
            app.UIAxes.YTickLabel = {'-0.1'; '0.0'; '0.1'; '0.2'; '0.3'; '0.4'; '0.5'; '0.6'; '0.7'; '0.8'; '0.9'; '1.0'; '1.1'; '1.2'; '1.3'; '1.4'; '1.5'};
            app.UIAxes.YMinorTick = 'on';
            app.UIAxes.LineWidth = 1;
            app.UIAxes.TitleFontSizeMultiplier = 2;
            app.UIAxes.Position = [10 273 576 401];

            % Create UIAxes2
            app.UIAxes2 = uiaxes(app.UIFigure);
            title(app.UIAxes2, 'Graphical Results')
            xlabel(app.UIAxes2, 'Time (s)')
            ylabel(app.UIAxes2, 'Displacement (m)')
            zlabel(app.UIAxes2, 'Z')
            app.UIAxes2.LabelFontSizeMultiplier = 1.3;
            app.UIAxes2.LineWidth = 1;
            app.UIAxes2.TitleFontSizeMultiplier = 2;
            app.UIAxes2.Position = [602 245 547 427];

            % Create BumpHeightmSliderLabel
            app.BumpHeightmSliderLabel = uilabel(app.UIFigure);
            app.BumpHeightmSliderLabel.HorizontalAlignment = 'right';
            app.BumpHeightmSliderLabel.Position = [228 205 96 22];
            app.BumpHeightmSliderLabel.Text = 'Bump Height (m)';

            % Create BumpHeightmSlider
            app.BumpHeightmSlider = uislider(app.UIFigure, 'range');
            app.BumpHeightmSlider.Limits = [0 0.25];
            app.BumpHeightmSlider.MajorTicks = [0 0.05 0.1 0.15 0.2 0.25];
            app.BumpHeightmSlider.MajorTickLabels = {'0', '0.05', '0.1', '0.15', '0.2', '0.25'};
            app.BumpHeightmSlider.ValueChangedFcn = createCallbackFcn(app, @BumpHeightmSliderValueChanged, true);
            app.BumpHeightmSlider.MinorTicks = [0 0.01 0.02 0.03 0.04 0.05 0.06 0.07 0.08 0.09 0.1 0.11 0.12 0.13 0.14 0.15 0.16 0.17 0.18 0.19 0.2 0.21 0.22 0.23 0.24 0.25];
            app.BumpHeightmSlider.Step = 0.01;
            app.BumpHeightmSlider.Position = [364 215 579 3];
            app.BumpHeightmSlider.Value = [0 0];

            % Create TireRadiusmSliderLabel
            app.TireRadiusmSliderLabel = uilabel(app.UIFigure);
            app.TireRadiusmSliderLabel.HorizontalAlignment = 'right';
            app.TireRadiusmSliderLabel.Position = [228 153 87 22];
            app.TireRadiusmSliderLabel.Text = 'Tire Radius (m)';

            % Create TireRadiusmSlider
            app.TireRadiusmSlider = uislider(app.UIFigure, 'range');
            app.TireRadiusmSlider.Limits = [0.25 0.6];
            app.TireRadiusmSlider.MajorTicks = [0.25 0.3 0.35 0.4 0.45 0.5 0.55 0.6];
            app.TireRadiusmSlider.MajorTickLabels = {'0.25', '0.3', '0.35', '0.4', '0.45', '0.5', '0.55', '0.6'};
            app.TireRadiusmSlider.ValueChangedFcn = createCallbackFcn(app, @TireRadiusmSliderValueChanged, true);
            app.TireRadiusmSlider.MinorTicks = [0.25 0.26 0.27 0.28 0.29 0.3 0.31 0.32 0.33 0.34 0.35 0.36 0.37 0.38 0.39 0.4 0.41 0.42 0.43 0.44 0.45 0.46 0.47 0.48 0.49 0.5 0.51 0.52 0.53 0.54 0.55 0.56 0.57 0.58 0.59 0.6];
            app.TireRadiusmSlider.Step = 0.01;
            app.TireRadiusmSlider.Position = [364 162 579 3];
            app.TireRadiusmSlider.Value = [0.25 0.25];

            % Create TireStiffnesskNmSliderLabel
            app.TireStiffnesskNmSliderLabel = uilabel(app.UIFigure);
            app.TireStiffnesskNmSliderLabel.HorizontalAlignment = 'right';
            app.TireStiffnesskNmSliderLabel.Position = [228 102 114 22];
            app.TireStiffnesskNmSliderLabel.Text = 'Tire Stiffness (kN/m)';

            % Create TireStiffnesskNmSlider
            app.TireStiffnesskNmSlider = uislider(app.UIFigure, 'range');
            app.TireStiffnesskNmSlider.Limits = [100 200];
            app.TireStiffnesskNmSlider.MajorTicks = [100 110 120 130 140 150 160 170 180 190 200];
            app.TireStiffnesskNmSlider.ValueChangedFcn = createCallbackFcn(app, @TireStiffnesskNmSliderValueChanged, true);
            app.TireStiffnesskNmSlider.MinorTicks = [100 105 110 115 120 125 130 135 140 145 150 155 160 165 170 175 180 185 190 195 200];
            app.TireStiffnesskNmSlider.Step = 5;
            app.TireStiffnesskNmSlider.Position = [364 112 579 3];
            app.TireStiffnesskNmSlider.Value = [100 100];

            % Create RunSimulationButton
            app.RunSimulationButton = uibutton(app.UIFigure, 'push');
            app.RunSimulationButton.ButtonPushedFcn = createCallbackFcn(app, @RunSimulationButtonPushed, true);
            app.RunSimulationButton.BackgroundColor = [0 1 0];
            app.RunSimulationButton.FontName = 'Unispace';
            app.RunSimulationButton.FontSize = 20;
            app.RunSimulationButton.FontColor = [1 1 1];
            app.RunSimulationButton.Position = [451 19 280 52];
            app.RunSimulationButton.Text = 'Run Simulation';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = suspension_sim_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end