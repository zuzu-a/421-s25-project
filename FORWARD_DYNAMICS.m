%% 1.1 -    Transfer function definition
%           In this section of the report, we define the coefficients of
%           the transfer function for all three different transfer
%           functions. We can deduce from the variables of the system that
%           all three transfer functions are SISO systems. These will be
%           fed into Simulink to give us the matrices for A, B, C, and D.
%           We will be using LQR, and possibly tuning, to get our required
%           tuning results.
clear; clc;

ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS                     = [0, 0, 6.2339, -9.5876, 292.43; 1, 3.7028, 28.6058, 88.9349, 71.1363];

%% 1.2 -    Unaugmented matrix
%           For clarity, we calculated the unaugmented matrix. However, we
%           found that it wasn't tracking good enough for our LQR
%           objective.

[A_fwd, B_fwd, C_fwd, D_fwd]    = tf2ss(ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS(1,:), ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS(2,:));
Q_fwd                           = diag([1/(200^2), 1/(50^2), 1/(100^2), 1/(100^2)]);
R_fwd                           = 0.01;
K_fwd                           = lqr(A_fwd, B_fwd, Q_fwd, R_fwd);

%% 1.3 -    Integral feedback, augmented state-space, LQR
%           We implemented an augmented state-space to complement our
%           integral feedback term for our system. We used LQR to minimize
%           a cost function.

% Augmented system matrices
A_fwd_aug   = [A_fwd, zeros(size(A_fwd,1),1); -C_fwd, 0];  
B_fwd_aug   = [B_fwd; 0];
C_fwd_aug   = [C_fwd 0];
D_fwd_aug   = D_fwd;

% Designing the Q matrix
q1_fwd      = 0.1;
q2_fwd      = 1;
q3_fwd      = 5;
q4_fwd      = 5;
q5_fwd      = 10;
Q_fwd_aug   = diag([q1_fwd, q2_fwd, q3_fwd, q4_fwd, q5_fwd]);

% Calculating our gains for our controller.
K_fwd_aug   = lqr(A_fwd_aug, B_fwd_aug, Q_fwd_aug, R_fwd);


%% 1.4 -    Initial conditions
%           Now we need to define initial conditions for our system.
INIT_COND_fwd = [0, 100, 0, 0];
