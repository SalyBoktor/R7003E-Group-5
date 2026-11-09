%% 3.1 Derive the Equations of Motion
disp('3.1 Deriving the Equations of Motion')

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
T_f = (b_f * ((diff(x_w,t) / l_w) - diff(theta_b, t)));
T_m = (K_t / R_m) * v_m - (K_e * K_t / R_m) * ((diff(x_w,t) / l_w) - diff(theta_b, t)); 

% Non-Linear EOM of Wheel Dynamics
eq1_left  = ((I_w / l_w) + l_w * m_b + l_w * m_w) * x_w_ddot;
eq1_right = - m_b * l_b * l_w * theta_b_ddot * cos(theta_b) + m_b * l_b * l_w * (theta_b_dot^2) * sin(theta_b) - T_f + T_m;
eq1 = eq1_left == eq1_right;

% Non-Linear EOM of Body Dynamics
eq2_left  = (I_b + m_b * l_b^2) * theta_b_ddot;
eq2_right = m_b * l_b * g * sin(theta_b) - m_b * l_b * x_w_ddot * cos(theta_b) - T_m + T_f;
eq2 = eq2_left == eq2_right;

%% 3.2 Linearize the Equations of Motion
disp('3.2 Linearizing the Equations of Motion')

% Choosing the Linearization points (all zero to keep the fella uppright)

% Linearized of Wheel Dynamics
eq1_lin = eq1_left == - m_b * l_b * l_w * theta_b_ddot(1) - T_f + T_m;
% Linearized EOM of Body Dynamics
eq2_lin = eq2_left == m_b * l_b * g * theta_b(t) - m_b * l_b * x_w_ddot(1) - T_m + T_f;

%% 3.3 Write the linearized Equations of Motion in State-Space form
disp('3.3 Linearizing the Equations of Motion in State-Space form')

k = K_e * K_t / R_m + b_f;
q = K_t / R_m;

gamma_11 = m_b * l_b;
gamma_12 = I_b + m_b * l_b^2;

gamma_21 = I_w / l_w + l_w * m_b + l_w * m_w;
gamma_22 = m_b * l_b * l_w;

a_11 = 0;
a_12 = k / l_w; 
a_13 = m_b * l_b * g;
a_14 = -k;

a_21 = 0; 
a_22 = -k / l_w;
a_23 = 0;
a_24 = k;

beta_1 = -q;
beta_2 = q;

u = v_m;

% Matrix for gamma
T = [gamma_11, gamma_12;
    gamma_21, gamma_22];

Alpha = [a_11, a_12, a_13, a_14;
         a_21, a_22, a_23, a_24];

Beta = [beta_1;
        beta_2];

T_inv = inv(T);

A_acc = T \ Alpha;

A = [0, 1, 0, 0;
    A_acc(1, :);
    0, 0, 0, 1;
    A_acc(2, :)];
B_acc = T \ Beta;

B = [0;
    B_acc(1);
    0; 
    B_acc(2)];

% Define C and D matrix
C = [0, 0, 1, 0];
D = 0;

%% 3.4 Determine the transfer function relative to system
disp('3.4 Computing the transfer function')

% State-Space System
sys = ss(A, B, C, D);

% State-Space to Transfer Function G(s)
[num, den] = ss2tf(A, B, C, D);
G = tf(num, den);

% Poles of the system (roots of the denominator)
system_poles = pole(G);

% Zeros of the system (roots of the numerator)
system_zeros = zero(G);

% check it
disp('Transfer Function G(s):');
G

%% 3.5 Design a PID controller stabilizing the TF
disp('3.5 Designing the PID controller stabilizing the TF')

%% 3.7 Check if everything is working as it should be
disp('3.7 Checking if everything is working as it should be')

%% 3.8 Convert the controller to the discrete domain
disp('3.8 Converting the controller to the discrete domain')