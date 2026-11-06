%% 3.1 Derive the Equations of Motion

% Dynamic symbols for later input
syms x_w(t) theta_b(t) v_m(t)

% velocity of wheel
theta_w_dot = diff(x_w, t)/l_w;
% velocity of body
theta_b_dot = diff(theta_b, t);
% velocity of motor
theta_m_dot = theta_w_dot - theta_b_dot;
% horizontal acceleration of wheel
x_w_ddot    = diff(x_w, t, 2);
% acceleration of body
theta_b_ddot = diff(theta_b, t, 2);

% Task 3.1.4: Motor Torque (T_m) and Friction Torque (T_f)

T_f = b_f * ((diff(x_w,t) / l_w) - diff(theta_b, t));
T_m = (K_t / R_m) * v_m - (K_e * K_t / R_m) * ((diff(x_w,t) / l_w) - diff(theta_b, t)); 

% First Equation:
eq1_left1 = (m_w + m_b + I_w/(l_w^2)) * x_w_ddot;
eq1_left2 = + m_b * l_b * theta_b_ddot * cos(theta_b) - m_b * l_b * (theta_b_dot^2) * sin(theta_b);
eq1_right = (T_m - T_f)/l_w;
eq1 = eq1_left - eq1_left2 == eq1_right;

%% 3.2 Linearize the Equations of Motion

% Choosing the Linearization points (all zero to keep the fella uppright)
theta_b(t) = 0;
theta_b_dot = 0;
theta_b_ddot = 0;

% Linearize the parts of the equation that is non-linear
eq1_lin_left2 = + m_b * l_b * theta_b_ddot;

% Final Linearized EOM
eq1_lin = eq1_left1 - eq1_lin_left2 == eq1_right;

%% 3.3 Write the linearized Equations of Motion in State-Space form



%% 3.4 Determine the transfer function relative to system

%% 3.5 Design a PID controller stabilizing the TF

%% 3.7 Check if everything is working as it should be

%% 3.8 Convert the controller to the discrete domain