%% Constants

% Physical constants
k_1   = 1 ; % growth rate of prey             
k_2   = 1 ; % mortality rate of predator  
k_3   = 1 ; % correlation coefficient between prey and predator
k_4   = 1 ; % correlation coefficient between predator and prey 
u     = 0 ; % input without control
x_ref = 5 ; % reference for prey population



%% Regulator parameters

% PD controller parameters
K_p   = 5   ; %K_p = 50 is better but violate constraints
K_d   = 7   ;

% Feedback Linearization parameters
K_1 = 4  ;
K_2 = 18 ;



%% Dynamics

[x1, x2] = meshgrid(-4:0.2:7, -4:0.2:7);

%Defining dynamic without control
x1dot  =  x1 - x2 .*x1       ; 
x2dot  =  x1.* x2 - x2 - (u) ;

%Defining dynamic with PD control
x1dotPD  =  x1 - x2 .*x1;
x2dotPD  =  x1.* x2 - x2 - (( (x_ref-x1).*K_p - K_d.*(x1 - x1.*x2) ) );

%Defining dynamic with nonlinear control
x1dotNon =  x1 - x2 .*x1; 
x2dotNon =  x1.* x2 - x2 + (  (-x1dotNon.*(1-x2)) + (x1.*x1.*x2)  - (x1.*x2) - (K_1.*(x1-x_ref)) - (K_2.*-x1dotNon)             );

%Lotka-Volterra system
f    = @(t,y) [y(1) - y(1)*y(2); y(1)*y(2) - y(2) - (u) ]; 
fPD  = @(t,y) [y(1) - y(1)*y(2); y(1)*y(2) - y(2) - (( (x_ref-y(1) )*K_p - K_d*( y(1) - y(1)*y(2) ) )) ]; 
fNon = @(t,y) [y(1) - y(1)*y(2); y(1)*y(2) - y(2) - (  ((-(K_1)*( y(1)-x_ref ))/y(1)) - ((1-y(2))*K_2)   - y(2)  + (y(1)*y(1)*y(2)) - ((y(1)-y(1)*y(2) )*(1-y(2)))        )/y(1)          ]; 



%% Simulation

%Simulating simulink model
sim('LotkaVolterra') ; 

%% Plotting states

%Plotting the trajectory from simulink model against each other

x=3;
y=0:0.001:10;
figure('Name','TrajectoryPlot');
hold on;
grid on
title('2-D Line Plot')
plot(PreyData.time(1:numel(PreyData.time)),PreyData.signals.values(1:numel(PreyData.time)), 'Color', [139/255, 69/255, 19/255],'LineWidth',1.5);
plot(PredatorData.time(1:numel(PredatorData.time)),PredatorData.signals.values(1:numel(PredatorData.time)),'k','LineWidth',1.5);
legend('Prey','Predator');
ylim([-2, 10]);
xlim([0, 30]);

%Plotting the data from input from simulink model
figure('Name','Input');
hold on;
grid on
title('Input 2-D Line Plot')
plot(uNon.time(1:numel(uNon.time)),uNon.signals.values(1:numel(uNon.time)), 'Color', [139/255, 69/255, 19/255],'LineWidth',1.5);
legend('PD Input');
ylim([-25, 120]);
xlim([0, 30]);


%% Phase Portrait without control

%Plotting vector field with quiver
figure
quiver(x1,x2,x1dot, x2dot)
xlim([0, 7]);
ylim([0, 7]);
hold on

%Calculate trajectories for different initial conditions
for y0=0:0.6:4
[ts, ys] = ode45(f,[0, 8], [y0/2, y0]);
plot(ys(:,1), ys(:,2))
end
hold off
xlabel('x')
ylabel('y')

%% Phase Portrait with PD control

%Plotting vector field with quiver
figure
quiver(x1,x2,x1dotPD, x2dotPD)
xlim([0, 7]);
ylim([-2, 7]);
hold on

%Calculate trajectories for different initial conditions
for y0=0:0.6:7
[ts, ys] = ode45(fPD,[0, 8], [y0/2, y0]);
plot(ys(:,1), ys(:,2))
end
hold off
xlabel('x')
ylabel('y')

%% Phase Portrait with nonlinear control

%Plotting vector field with quiver
figure
quiver(x1,x2,x1dotNon, x2dotNon)
xlim([0, 7]);
ylim([0, 7]);
hold on

%Calculate trajectories for different initial conditions
for y0=0:0.6:7
[ts, ys] = ode45(fNon,[0, 8], [y0/2, y0]);
plot(ys(:,1), ys(:,2))
end
hold off
xlabel('x')
ylabel('y')