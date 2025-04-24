%% 1.1 -    Transfer function definition
%           In this section of the report, we define the coefficients of
%           the transfer function for all three different transfer
%           functions. We can deduce from the variables of the system that
%           all three transfer functions are SISO systems. These will be
%           fed into Simulink to give us the matrices for A, B, C, and D.
%           We will be using LQR, and possibly tuning, to get our required
%           tuning results.

ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS                     = [0, 0, 0, 3.6083; 1, 0.8229, 0.1394, 0];
LATERAL_ROTATIONAL_TRANSFER_FUNCTION_COEFFICIENTS           = [0, 0, 6.2339, -9.5876, 292.43; 1, 3.7028, 28.6058, 88.9349, 71.1363];


[A_altitude, B_altitude, C_altitude, D_altitude]            = tf2ss(ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS(1,:), ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS(2,:));
[A_rotational, B_rotational, C_rotational, D_rotational]    = tf2ss(LATERAL_ROTATIONAL_TRANSFER_FUNCTION_COEFFICIENTS(1,:), LATERAL_ROTATIONAL_TRANSFER_FUNCTION_COEFFICIENTS(2,:));


Q_altitude      = diag([0, 0, 1]);
R_altitude      = [0.01];

Q_rotational    = [1];
R_rotational    = [0.01];

Q_forward       = [1];
R_forward       = [0.01];

K_altitude      = lqr(A_altitude, B_altitude, Q_altitude, R_altitude);
K_rotational    = lqr(A_rotational, B_rotational, Q_rotational, R_rotational);

%Q_forward       = lqr(A_rotational, B_rotational, Q_rotational, R_rotational);


A_altitude =        [
                    0           , 1         , 0; 
                    0           , 0         , 1;
                    0           , -0.1394   , -0.8229
                    ];

B_altitude      =   [
                    0; 
                    0; 
                    3.6083
                    ];

C_altitude      =   [1, 0, 0];

D_altitude      =   [0];


A_rotational    =   [
                    0           , 1         , 0; 
                    0           , 0         , 1;
                    -1.5901     , -0.6254   , -2.5426
                    ];

B_rotational    =   [
                    0
                    0
                    0.1257
                    ];

C_rotational    = [1, 0, 0];

D_rotational    = [0];