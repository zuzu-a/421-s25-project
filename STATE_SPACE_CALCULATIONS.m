ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS = [0, 0, 0, 3.6083; 1, 0.8229, 0.1394, 0];

[A_altitude, B_altitude, C_altitude, D_altitude] = tf2ss(ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS(1,:), ALTITUDE_TRANSFER_FUNCTION_COEFFICIENTS(2,:));

Q_altitude = [1];
R_altitude = [0.01];

K_altitude = lqr(A_altitude, B_altitude, Q_altitude, R_altitude);