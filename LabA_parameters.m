fprintf('loading the physical parameters...');
% acceleration due to gravity
g = 9.8;
b_f = 0;
% Mass of the Body
m_b = 0.381 .* 10.^3;
% Radius of the Body
l_b = 0.112;
% moment of inertia of the Body
I_b = 0.00616 .* 10.^3;
% Mass of the wheel
m_w = 0.036 .* 10.^3;
% radius of the wheel 
l_w = 0.021;
% moment of inertia of the wheel
I_w = 0.00000746 .* 10.^3;
% motor electrical resistance 
R_m = 4.4;
% motor electrical inductance 
L_m = 0;
% motor viscous coefficient
b_m = 0;
% Motor back emf constant
K_e = 0.444;
% Motor torque constant
K_t = 0.470;


%% 3.1 Derive the Equations of Motion


%% 3.3 Write the linearized Equations of Motion in State-Space form

%% 3.4 Determine the transfer function relative to system

%% 3.5 Design a PID controller stabilizing the TF

%% 3.7 Check if everything is working as it should be

%% 3.8 Convert the controller to the discrete domain
