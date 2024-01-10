% System-Control-for-Gripper (Time Response without Controller)
% System parameters 
K = 1; 
theta2_desired = 45; 

% Initial conditions
initial_conditions = [10, 15, 5, theta2_desired, 13, 2, 3, 5, 4];

% Definition of the equilibrium equation (equation 5)
eq = @(t, Ta, theta_a1, theta2, theta2_desired, theta2_dot, zeta1, zeta2, xi1, xi2) ...
    [Ta - K * (theta2 - theta2_desired) * theta2_dot + Kp * (theta2_desired - theta2) + Ki * (theta2_desired - theta2) * (t - t(1)) + Kd * (0 - theta2_dot); % Equation for Ta
    theta_a1; % Equation for theta_a1
    theta2_dot; % Equation for theta2
    0; % Equation for theta2_desired (specified as desired value)
    zeta1 - xi1; % Equation for zeta1
    zeta2 - xi2; % Equation for zeta2
    0; % Equation for xi1 
    0; % Equation for xi2 
    0; % 
];

% Numerical resolution using ode45
t_span = [0, 10]; % Time interval

% Solve the equations using ode45
[T, Y] = ode45(@(t, y) your_system_equations(t, y, eq), t_span, initial_conditions);

% Plot the results
figure;
plot(T, Y(:, 3), 'LineWidth', 2); % Plot theta2 as a function of time
xlabel('Time');
ylabel('\theta_2 [ deg ] ');
title('Response of the Second Phalanx Angle System Over Time.');

% Find the transfer function
syms s; % Laplace complex variable
Y = laplace(eq(0, 0.5, pi/3, pi/4, 0.2, 0.1, 0.3, 0.4, 0.5), s); % Laplace of the equations

% Extract the transfer function
G = Y / laplace(0.1, s); % Assuming the input is 0 to find the transfer function

% Display the transfer function
disp('Transfer Function:');
disp(G);

% Function representing the system of differential equations
function dydt = your_system_equations(t, y, eq)
    % Breakdown of state variables
    Ta = y(1);
    theta_a1 = y(2);
    theta2 = y(3);
    theta2_desired = y(4);
    theta2_dot = y(5);
    zeta1 = y(6);
    zeta2 = y(7);
    xi1 = y(8);
    xi2 = y(9);

    % Apply the equilibrium equation
    eq_result = eq(t, Ta, theta_a1, theta2, theta2_desired, theta2_dot, zeta1, zeta2, xi1, xi2);

    % Save the results in the vector of rate changes
    dydt = eq_result;
end

