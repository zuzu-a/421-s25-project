%% 2.1 -    Transfer function definition
%           In this section of the report, we define the coefficients of
%           the transfer function for all three different transfer
%           functions. We can deduce from the variables of the system that
%           all three transfer functions are SISO systems. These will be
%           fed into Simulink to give us the matrices for A, B, C, and D.
%           We will be using LQR, and possibly tuning, to get our required
%           tuning results.

ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS                     = [0, 0, 0, 0.1257; 1, 2.5426, 0.6254, 1.5901];

%% 2.2 -    Unaugmented matrix
%           For clarity, we calculated the unaugmented matrix. However, we
%           found that it wasn't tracking good enough for our LQR
%           objective.

[A_rot, B_rot, C_rot, D_rot]            = tf2ss(ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS(1,:), ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS(2,:));
Q_rot = diag([1/(200^2), 1/(50^2), 1/(100^2)]);
R_rot = 0.01;
K_rot      = lqr(A_rot, B_rot, Q_rot, R_rot);

%% 2.3 -    Integral feedback, augmented state-space, LQR
%           We implemented an augmented state-space to complement our
%           integral feedback term for our system. We used LQR to minimize
%           a cost function.

% Augmented system matrices
A_rot_aug   = [A_rot, zeros(size(A_rot,1),1); -C_rot, 0];  
B_rot_aug   = [B_rot; 0];
C_rot_aug   = [C_rot 0];
D_rot_aug   = D_rot;

% Designing the Q matrix
q1_rot      = 10;
q2_rot      = 100;
q3_rot      = 1;
q4_rot      = 100;
Q_rot_aug   = diag([q1_rot, q2_rot, q3_rot, q4_rot]);

% Calculating our gains for our controller.
K_rot_aug   = lqr(A_rot_aug, B_rot_aug, Q_rot_aug, R_rot);

%% 1.4 -    Initial conditions
%           Now we need to define initial conditions for our system.
INIT_COND_rot = [0, 0, 0, 0];
