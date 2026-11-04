%% 3.1 Derive the Equations of Motion

syms x_b(t) y_b(t) theta_b(t) F_x(t) F_y(t) T_m(t) T_f(t) 
syms m_b g I_b l_b

eq4 = m_b * diff(x_b, t, 2) == F_x;
eq5 = m_b * diff(y_b, t, 2) == F_y - m_b * g;
eq6 = I_b * diff(theta_b, t, 2) == -T_m + T_f + F_y*l_b*sin(theta_b) - F_x*l_b*cos(theta_b);


%% 3.2 Linearize the Equations of Motion


%% 3.3 Write the linearized Equations of Motion in State-Space form

%% 3.4 Determine the transfer function relative to system

%% 3.5 Design a PID controller stabilizing the TF

%% 3.7 Check if everything is working as it should be

%% 3.8 Convert the controller to the discrete domain