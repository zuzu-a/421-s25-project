%% 1.1 -    Transfer function definition
%           In this section of the report, we define the coefficients of
%           the transfer function for all three different transfer
%           functions. We can deduce from the variables of the system that
%           all three transfer functions are SISO systems. These will be
%           fed into Simulink to give us the matrices for A, B, C, and D.
%           We will be using LQR, and possibly tuning, to get our required
%           tuning results.

ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS                     = [0, 0, 0, 3.6083; 1, 0.8229, 0.1394, 0];

%% 1.2 -    Unaugmented matrix
%           For clarity, we calculated the unaugmented matrix. However, we
%           found that it wasn't tracking good enough for our LQR
%           objective.

[A_alt, B_alt, C_alt, D_alt]            = tf2ss(ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS(1,:), ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS(2,:));
Q_alt = diag([1/(200^2), 1/(50^2), 1/(100^2)]);
R_alt = 1;
K_alt      = lqr(A_alt, B_alt, Q_alt, R_alt);

%% 1.3 -    Integral feedback, augmented state-space, LQR
%           We implemented an augmented state-space to complement our
%           integral feedback term for our system. We used LQR to minimize
%           a cost function.

% Augmented system matrices
A_alt_aug   = [A_alt, zeros(size(A_alt,1),1); -C_alt, 0];  
B_alt_aug   = [B_alt; 0];
C_alt_aug   = [C_alt 0];
D_alt_aug   = D_alt;

% Designing the Q matrix
q1_alt      = 10;
q2_alt      = 1;
q3_alt      = 5;
q4_alt      = 15;
Q_alt_aug   = diag([q1_alt, q2_alt, q3_alt, q4_alt]);

% Calculating our gains for our controller.
K_alt_aug   = lqr(A_alt_aug, B_alt_aug, Q_alt_aug, R_alt);

%% 1.4 -    Initial conditions
%           Now we need to define initial conditions for our system.
INIT_COND_alt = [0, 0, 0, 0];
